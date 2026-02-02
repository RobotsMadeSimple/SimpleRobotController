using System;
using System.Collections.Generic;
using System.Linq;

public static class BitTools
{
    public static int BytesToNumber(IEnumerable<byte> bytes)
    {
        int value = 0;
        int shift = 0;
        foreach (var b in bytes)
        {
            value |= b << shift;
            shift += 8;
        }
        return value;
    }

    public static List<int> ByteToBits(byte value)
    {
        var bits = new List<int>(8);
        for (int i = 7; i >= 0; i--)
            bits.Add((value >> i) & 1);
        return bits;
    }

    public static List<byte> NumberToBytes(int value, int count)
    {
        var bytes = new List<byte>(count);
        for (int i = 0; i < count; i++)
        {
            bytes.Add((byte)(value & 0xFF));
            value >>= 8;
        }
        return bytes;
    }

    public static byte BitsToByte(IList<int> bits)
    {
        byte value = 0;
        for (int i = 0; i < 8; i++)
            if (bits[i] == 1)
                value |= (byte)(1 << (7 - i));
        return value;
    }

    public static List<byte> NumberToSignedBytes(int value)
    {
        return BitConverter.GetBytes(value).ToList(); // little-endian
    }
}
