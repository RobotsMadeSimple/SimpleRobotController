using System;

namespace Controller.RobotControl.Camera.WebRtc
{
    /// <summary>
    /// Decides how fast the phone should send, from what the receiver actually sees — so the stream
    /// settles just below what the link can carry instead of overrunning it.
    ///
    /// <para><b>Why.</b> The phone's encoder runs at a fixed bitrate (the app pins it, because
    /// SIPSorcery gives libwebrtc no usable bandwidth feedback). When the Wi-Fi link can't carry that
    /// rate its queue overflows in bursts — whole runs of packets, key frames included — faster than
    /// resends can repair, and the picture freezes. Random air loss is different: resends fix it, and
    /// lowering the bitrate wouldn't help.</para>
    ///
    /// <para><b>Rule (AIMD, once a second).</b> Congested = a packet stayed lost after its resends,
    /// the decoder fell behind, or more than <see cref="LossFraction"/> of packets went missing in the
    /// first place. Then the cap drops to <see cref="Decrease"/> of the rate actually arriving (never
    /// below <see cref="FloorKbps"/> or under <see cref="MaxCut"/> of the old cap, at most once a second). After <see cref="CleanBeforeRaiseMs"/>
    /// with almost no loss it rises by <see cref="Increase"/> every <see cref="RaiseEveryMs"/>, and once
    /// it is well above what the phone is sending anyway the cap is lifted.</para>
    ///
    /// The phone applies the cap with <c>RTCRtpSender.setParameters</c> (maxBitrate), under the user's
    /// own bitrate setting, and shows it. Not thread-safe; times are caller-supplied ms.
    /// </summary>
    public sealed class BitrateGovernor
    {
        public const int FloorKbps = 1500;
        public const double LossFraction = 0.03;
        public const double QuietLossFraction = 0.01;
        public const double Decrease = 0.8;
        public const double Increase = 1.08;
        /// <summary>One decrease never cuts more than this share (the arriving rate dips while the link is choking).</summary>
        public const double MaxCut = 0.5;
        public const int CleanBeforeRaiseMs = 3000;
        public const int RaiseEveryMs = 1000;
        public const int MinDecreaseGapMs = 1000;

        /// <summary>The current cap in kbps, or null when the phone may send at its own setting.</summary>
        public int? CapKbps { get; private set; }
        /// <summary>How many times the cap has been lowered.</summary>
        public int Decreases { get; private set; }
        /// <summary>The highest arriving rate seen recently (kbps), to know when a cap no longer binds.</summary>
        public double PeakRxKbps { get; private set; }

        private long _lastDecreaseMs = long.MinValue / 2;
        private long _lastRaiseMs = long.MinValue / 2;
        private long _cleanSinceMs = -1;

        /// <summary>
        /// One second's observations. <paramref name="rxKbps"/>: payload rate arriving (resends included);
        /// <paramref name="received"/> / <paramref name="missing"/>: packets that arrived / holes that opened;
        /// <paramref name="lost"/>: packets given up on after their resends; <paramref name="decoderOverflows"/>:
        /// times the decoder had to drop queued frames. Returns true when the cap changed.
        /// </summary>
        public bool Update(long nowMs, double rxKbps, long received, long missing, long lost, long decoderOverflows)
        {
            if (received <= 0 && missing <= 0) return false;   // nothing flowing: nothing to judge
            PeakRxKbps = Math.Max(rxKbps, PeakRxKbps * 0.98);
            double lossFrac = missing / (double)Math.Max(1, received + missing);
            bool congested = lost > 0 || decoderOverflows > 0 || lossFrac > LossFraction;

            if (congested)
            {
                _cleanSinceMs = -1;
                if (nowMs - _lastDecreaseMs < MinDecreaseGapMs) return false;
                double basis = Math.Min(CapKbps ?? double.MaxValue, Math.Max(rxKbps, 1));
                double next = basis * Decrease;
                if (CapKbps is int was) next = Math.Max(next, was * MaxCut);
                next = Math.Max(FloorKbps, next);
                if (CapKbps is int cur && next >= cur) return false;
                CapKbps = (int)next;
                Decreases++;
                _lastDecreaseMs = nowMs;
                return true;
            }

            if (lossFrac > QuietLossFraction) { _cleanSinceMs = -1; return false; }
            if (_cleanSinceMs < 0) _cleanSinceMs = nowMs;
            if (CapKbps is not int cap) return false;
            if (nowMs - _cleanSinceMs < CleanBeforeRaiseMs || nowMs - _lastRaiseMs < RaiseEveryMs) return false;
            _lastRaiseMs = nowMs;
            int raised = (int)Math.Max(cap + 250, cap * Increase);
            // The phone sends well under the cap anyway (its own setting, or a quiet scene): lift it.
            CapKbps = raised > PeakRxKbps * 1.5 + 500 ? null : raised;
            return true;
        }
    }
}
