"""
Per-step-type field reference for ProgramStep, served by the `robot_step_schema`
tool so an agent can pull only the step types it needs instead of carrying the
whole 250-field model in context.

Mirrors RobotControl/Models.cs (ProgramStep, StepType). When a field is added
there, add it here — nothing enforces the link, so a missing entry shows up as
an agent writing a step the executor silently ignores.
"""

# Fields legal on every step regardless of type.
COMMON = {
    "id":   "string, required, unique within the program. Any stable id works; a uuid4 is typical.",
    "name": "string, optional label shown in the builder list.",
    "type": "string, required. One of the StepType names below (exact casing).",
    "expressions": (
        "object, optional. Maps any numeric field name on this step (camelCase, e.g. "
        '"speed", "waitMs", "offsetZ") to a math expression string evaluated at run time. '
        "Overrides the literal value in that field. Variable references REQUIRE the $ sigil "
        '- "$baseSpeed * 2". A bare word evaluates to 0, not to a variable.'
    ),
}

# Shared by MoveL / MoveJ / JumpL / JumpJ / ThreadMove.
MOVE_TARGET = {
    "pointName": "string. Name of a saved point (see the robot_points tool).",
    "pointNameExpr": (
        "string, overrides pointName. Two resolutions: an expression that is ONLY an indexed "
        'Point list ("$pts[$i]" or "{$pts[0]}") yields those coordinates directly; anything '
        'else is interpolated to text naming a saved point ("$target", "{$binPrefix}{$index}"). '
        "Re-resolved every execution, so assigning the referenced variables retargets the move."
    ),
    "gridPoint":  'object {gridId, row, col} or expression form - overrides pointName with a grid cell.',
    "stackPoint": 'object {stackId, index} - overrides pointName with a 1-D indexed position.',
    "speed": "number. Linear mm/s for MoveL/JumpL, joint deg/s for MoveJ/JumpJ.",
    "accel": "number. mm/s^2 (linear) or deg/s^2 (joint).",
    "decel": "number. Same units as accel.",
    "blend": "bool. Round the corner into the next move instead of stopping.",
    "blendRadius": "number, mm. Overrides the program's current default blend radius.",
    "offsetX/Y/Z":     "number, mm. Added to the target point.",
    "offsetRX/RY/RZ":  "number, deg. Added to the target orientation.",
    "toolOffsetX/Y/Z": "number, mm. Local tool offset applied at execution time.",
    "toolOffsetRX/RY/RZ": "number, deg.",
    "overrideX/Y/Z":   "number, mm. Replaces the calculated axis value (base + offset) outright.",
    "overrideRX/RY/RZ": "number, deg. Same, for orientation.",
    "localName": "string. Per-step local-frame override for this move.",
}

CONDITION_GROUP = (
    'object {combinator: "ALL"|"ANY", items: [{id, left, operator, right}]}. '
    "`left` and `right` are expression strings (use $ for variables); `operator` is one of "
    "== != < <= > >=."
)

STEPS = {
    # ── Motion ────────────────────────────────────────────────────────────
    "MoveL": {
        "what": "Linear (straight-line) move to a target.",
        "fields": MOVE_TARGET,
    },
    "MoveJ": {
        "what": "Joint-interpolated move. Joints move proportionally; the TCP path is not straight.",
        "fields": MOVE_TARGET,
    },
    "JumpL": {
        "what": "Lift, traverse, lower - a MoveL with an arch over the target.",
        "fields": {**MOVE_TARGET,
                   "jumpZ": "number, mm. Height used for both lift and lower legs.",
                   "jumpZStart": "number, mm. Overrides the lift leg only.",
                   "jumpZEnd": "number, mm. Overrides the lower leg only."},
    },
    "JumpJ": {
        "what": "Same arch as JumpL, joint-interpolated.",
        "fields": {**MOVE_TARGET,
                   "jumpZ": "number, mm.", "jumpZStart": "number, mm.", "jumpZEnd": "number, mm."},
    },
    "ThreadMove": {
        "what": "Coordinated Z + rotation move for tapping/threading.",
        "fields": {"threadDistance": "number, mm. Total depth.",
                   "threadPitch": "number, mm per revolution.",
                   "threadPeck": "bool. Peck-drill instead of one continuous pass.",
                   "threadPeckDepth": "number, mm per peck.",
                   "threadReverseOut": "bool. Unscrew back out on completion."},
    },
    "SetSpeedL": {
        "what": "Set the default linear speed/accel/decel for subsequent moves.",
        "fields": {"speed": "number, mm/s.", "accel": "number, mm/s^2.", "decel": "number, mm/s^2."},
    },
    "SetSpeedJ": {
        "what": "Set the default joint speed/accel/decel for subsequent moves.",
        "fields": {"speed": "number, deg/s.", "accel": "number, deg/s^2.", "decel": "number, deg/s^2."},
    },
    "SetBlendRadius": {
        "what": "Set the program's default blend radius for subsequent blended moves.",
        "fields": {"blendRadius": "number, mm."},
    },
    "SetTool": {
        "what": "Activate a saved tool (TCP offset) by name.",
        "fields": {"toolName": 'string. Empty or "None" clears the tool.'},
    },
    "SetLocal": {
        "what": "Activate a saved local coordinate frame by name.",
        "fields": {"localName": "string."},
    },
    "ClearLocal": {"what": "Return to the base frame.", "fields": {}},
    "RunHoming": {"what": "Run the homing sequence. Blocks until homed.", "fields": {}},

    # ── IO ────────────────────────────────────────────────────────────────
    "SetOutput": {
        "what": "Drive a digital output (gripper, valve, signal lamp).",
        "fields": {
            "outputNumber": "int. Output index on the card.",
            "outputValue": "bool. Target state.",
            "outputCard": 'string. Which IO device - e.g. "STB" or a relay/nano card.',
            "outputNanoId": "string. Device id when outputCard names a Nano edge device.",
            "pulseMs": "int. Set to outputValue for this long, then invert. 0/omitted = hold.",
            "pulseBlocking": "bool. With pulseMs > 0, block until the pulse completes.",
        },
    },

    # ── Flow ──────────────────────────────────────────────────────────────
    "Wait": {
        "what": "Pause for a duration, or until a condition holds.",
        "fields": {
            "waitMode": '"duration" (default) | "condition".',
            "waitMs": "int, ms. Used when waitMode is duration.",
            "waitCondition": CONDITION_GROUP,
            "waitTimeoutMs": "int, ms. Give up waiting for the condition after this long.",
            "waitTimeoutVariableName": "string. Variable set to 1 if the wait timed out, else 0.",
        },
    },
    "Loop": {
        "what": "Repeat nested steps by count, over a list, or while a condition holds.",
        "fields": {
            "loopMode": '"count" (default) | "forEach" | "while".',
            "loopCount": "int. 0 = infinite. Used in count mode.",
            "loopSteps": "array of ProgramStep. The loop body.",
            "forEachVariableName": "string. Name of the list variable to iterate, any elementType.",
            "forEachValueVariableName": ("string. Variable that receives each element. Only a "
                                         "Number or Boolean list has a scalar to give it; on a "
                                         "Point or Record list it receives the index instead."),
            "forEachIndexVariableName": "string. Variable that receives the 0-based index.",
            "loopWhileCondition": CONDITION_GROUP,
        },
    },
    "IfCondition": {
        "what": "Branch. Evaluates condition, then each elseIf in order, then else.",
        "fields": {
            "condition": CONDITION_GROUP,
            "ifSteps": "array of ProgramStep.",
            "elseIfBranches": "array of {id, condition: ConditionGroup, steps: [ProgramStep]}.",
            "elseSteps": "array of ProgramStep.",
        },
    },
    "Label":     {"what": "Named jump target.", "fields": {"labelId": "string.", "labelName": "string."}},
    "GoToLabel": {"what": "Jump to a Label step.", "fields": {"labelId": "string.", "labelName": "string."}},
    "CallRoutine": {
        "what": "Run another built program saved with isRoutine=true, then return.",
        "fields": {"routineName": "string.", "routineId": "string."},
    },
    "PauseProgram": {"what": "Halt until the operator resumes.", "fields": {}},

    # ── Variables & status ────────────────────────────────────────────────
    "SetVariable": {
        "what": "Assign a program variable from an expression.",
        "fields": {
            "variableName": "string. Must match a variable declared on the program.",
            "variableExpr": (
                'string. Math expression for numeric/boolean variables ("$count + 1"); for a string '
                'variable it is interpolated as a template ("bin{$index + 1}"). $ is required.'
            ),
        },
    },
    "StatusUpdate": {
        "what": "Post a message to the program log / operator display.",
        "fields": {
            "statusMessage": "string, interpolated. Supports $name and {$expr}.",
            "statusWarning": "string, interpolated.",
            "statusError":   "string, interpolated. Setting this faults the program.",
            "statusSeverity": '"Info" | "Warning" | "Error".',
        },
    },
    "StopwatchControl": {
        "what": "Start/stop/reset a stopwatch variable (value is elapsed ms).",
        "fields": {"stopwatchAction": '"Start" | "Stop" | "Reset".',
                   "stopwatchVariableName": "string. Must be a variable with isStopwatch=true."},
    },

    # ── Background programs ───────────────────────────────────────────────
    "StartBackground": {
        "what": "Launch a program saved with isBackground=true alongside this one. Motion/tool/homing steps are skipped in background programs.",
        "fields": {"backgroundProgramName": "string.", "backgroundProgramId": "string."},
    },
    "StopBackground":    {"what": "Stop a running background program.",
                          "fields": {"backgroundProgramName": "string.", "backgroundProgramId": "string."}},
    "WaitForBackground": {"what": "Block until a background program finishes.",
                          "fields": {"backgroundProgramName": "string.", "backgroundProgramId": "string."}},

    # ── Vision & imaging ──────────────────────────────────────────────────
    "RunVision": {
        "what": "Run a saved vision program and write its inspection results into variables.",
        "fields": {
            "visionProgramId": "string.", "visionProgramName": "string.",
            "visionZoneId": "string. Restrict to one zone.",
            "visionZoneVar": "string. Variable naming the zone at run time.",
            "visionOutputs":  "array of {inspectionId, countVar?, pointsVar?, detectedVar?} - blob inspections.",
            "colorOutputs":   ("array of {inspectionId, coverageVar?, passedVar?, cellsVar?, cellsPassedVar?}. "
                               "coverageVar is whole-zone %. On a zone with a grid, passedVar means every cell "
                               "passed, cellsVar takes an object-list variable filled with one record per cell "
                               "(row, col, index, coverage, passed), and cellsPassedVar gets the pass count."),
            "polygonOutputs": "array of {inspectionId, countVar?, foundVar?, angleVar?, centerXVar?, centerYVar?}.",
            "arucoOutputs":   "array of {inspectionId, countVar?, foundVar?, firstIdVar?, firstCenterXVar?, firstCenterYVar?}.",
        },
    },
    "CaptureImage": {
        "what": "Grab a frame into an image variable (isImage=true) for later send/save.",
        "fields": {"captureImageVariableName": "string.", "captureImageCameraId": "string."},
    },
    "SaveImage": {
        "what": "Write a camera frame to disk.",
        "fields": {"saveImagePath": 'string, interpolated. Built-in $time_ms is available - "captures/$time_ms.jpg".',
                   "saveImageCameraId": "string."},
    },

    # ── HTTP ──────────────────────────────────────────────────────────────
    "HttpRequest": {
        "what": "POST a JSON body to a URL and optionally load response values into variables.",
        "fields": {
            "jsonUrl": "string.",
            "jsonWaitForResponse": "bool.",
            "jsonTimeoutMs": "int, ms.",
            "jsonOutbound": 'array of {key, expr, imageVar?, listVar?}. Set exactly one per row: expr is '
                            'evaluated to a number, imageVar sends a base64 JPEG, listVar sends a whole list '
                            'variable as a JSON array (booleans as true/false, numbers as numbers, '
                            'points/objects as objects).',
            "jsonInbound":  "array of {key, variableName} - response key -> variable. A JSON array mapped onto "
                            "a list variable replaces the list, keeping the list's declared element type; read "
                            "how many came back with $name.length.",
            "jsonImageOutbound": "array of {key, variableName} - image variable sent as base64.",
        },
    },
    "HttpReceive": {
        "what": "Block until an inbound HTTP post arrives on a named endpoint, then load its values.",
        "fields": {"httpReceiveName": "string. Endpoint name.",
                   "httpReceiveTimeoutMs": "int, ms.",
                   "httpReceiveInbound": "array of {key, variableName}."},
    },

    # ── Aux axis ──────────────────────────────────────────────────────────
    "AuxMove": {
        "what": "Move an auxiliary stepper axis a fixed amount.",
        "fields": {
            "auxDeviceId": "string.", "auxAxisIndex": "int.",
            "auxSteps": "int. Signed; positive = CW. Ignored when auxDistance is set.",
            "auxDistance": "number. Physical-unit move; requires auxUnit.",
            "auxUnit": '"mm" (linear) | "deg" (rotary). Also switches velocity/accel/decel to those units.',
            "auxVelocity": "number. steps/s, or unit/s when auxUnit is set.",
            "auxAccel": "number.", "auxDecel": "number.",
            "auxWaitForDone": "bool, default true. Block until the move finishes.",
            "auxAbsolute": "bool. Treat the target as absolute rather than relative.",
        },
    },
    "AuxContinuous": {
        "what": "Run an aux axis continuously until AuxStop.",
        "fields": {"auxDeviceId": "string.", "auxAxisIndex": "int.",
                   "auxVelocity": "number.", "auxAccel": "number.",
                   "auxImmediate": "bool. Skip the accel ramp."},
    },
    "AuxStop":   {"what": "Ramp an aux axis down to a stop.",
                  "fields": {"auxDeviceId": "string.", "auxAxisIndex": "int.",
                             "auxDecel": "number.", "auxImmediate": "bool. Stop without ramping."}},
    "AuxEnable": {"what": "Enable or disable aux motor drivers.",
                  "fields": {"auxDeviceId": "string.", "auxAxisIndex": "int.", "auxEnable": "bool."}},

    # ── CNC ───────────────────────────────────────────────────────────────
    "CncProgram": {
        "what": "Run a DXF-derived toolpath. Steps are generated at run time from cncSpec.",
        "fields": {"cncDxfFile": "string.", "cncSafeZ": "number, mm.",
                   "cncSpec": "object. Toolpath spec built by the CNC builder in the app.",
                   "cncProgramSteps": "array of ProgramStep. Legacy - baked steps from older app versions."},
    },
}

# Step types the executor refuses to run inside a background program.
BACKGROUND_FORBIDDEN = {
    "MoveL", "MoveJ", "JumpL", "JumpJ", "ThreadMove", "CncProgram",
    "SetTool", "RunHoming", "SetSpeedL", "SetSpeedJ", "SetBlendRadius",
}

# Fields whose value is a nested step list, for recursive walks.
NESTED_STEP_FIELDS = ("loopSteps", "ifSteps", "elseSteps", "cncProgramSteps")

VARIABLE_FIELDS = {
    "id":          "string, required, unique.",
    "name":        "string, required. The name used after $ in expressions.",
    "value":       "number. Initial value (also the current value for booleans: 0/1).",
    "valueExpression": ("string. Expression giving the initial value instead of `value`, for "
                        "Number and Boolean variables only. Evaluated once at program start "
                        "(and on each routine entry) against the variables declared ABOVE this "
                        "one plus IO — variables initialise in declaration order. For a "
                        "boolean any non-zero result is True, so \"$count > 5\" works. Set "
                        "`value` as well: it is the fallback if the expression cannot be "
                        "evaluated."),
    "items":       ("array of {field: number}. Makes this a list variable, counted with "
                    "$name.length. Every element is a record of named numbers whatever the "
                    "element type: a Number or Boolean element holds its scalar under the "
                    "reserved key `value` (a boolean as 0/1), a Point element under "
                    "x/y/z/rx/ry/rz. Fields are numbers only. Point and Record lists are "
                    "usually declared as [] and filled at run time by RunVision."),
    "elementType": ("string, one of Number | Boolean | Point | Record. Goes with `items` and "
                    "decides how the list is read: Number and Boolean answer a bare $name[i] "
                    "(a Boolean as the 0/1 it is stored as, printed True/False when "
                    "interpolated); Point answers $name[i].x and positional $name[i][2] "
                    "(0=x 1=y 2=z 3=rx 4=ry 5=rz), and can be a move target; Record answers "
                    "$name[i].field. Absent reads as Record."),
    # Superseded by items/elementType. Still accepted so programs saved before the list
    # types were unified keep loading; a program re-saved by the app carries items instead.
    "values":      "deprecated. Legacy form of items with elementType Number.",
    "points":      "deprecated. Legacy form of items with elementType Point.",
    "objects":     "deprecated. Legacy form of items with elementType Record.",
    "description": "string.",
    "isBoolean":   "bool.",
    "isString":    "bool. Pairs with stringValue.",
    "stringValue": "string. Initial value when isString.",
    "isImage":     "bool. Holds a base64 JPEG, populated by CaptureImage.",
    "isGlobal":    "bool. Shared across all concurrently running programs.",
    "isPersistent": "bool. Value saved on finish, restored next run.",
    "isStopwatch": "bool. Value is elapsed ms, driven by StopwatchControl.",
    "displayOnMonitor": "bool. Show the live value on the monitor page.",
}
