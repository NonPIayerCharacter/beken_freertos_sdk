
:link_to_translation:`zh_CN:[中文]`

Instruction description
========================

+--------------------+--------------------------+-------------------------------------+
| Type               | Command format           | Description                         |
+====================+==========================+=====================================+
| Query command      | AT+<Command Name>?       | Return the current parameter value  |
+--------------------+--------------------------+-------------------------------------+
| Execution command  | AT+<Command Name>=<...>  | Set user-defined parameter values   |
+--------------------+--------------------------+-------------------------------------+

Not every AT command has the above two types of commands. The input parameters in the command currently only support string parameters and integer parameters.

- Parameters within angle brackets < > cannot be omitted.
- Parameters within square brackets [ ] can be omitted. If omitted, the default value will be used.
- The default baud rate for AT commands is 115200.
- The length of each AT command should not exceed 256 bytes.
- AT commands end with a new line (CR-LF), so the serial port tool should be set to "new line mode".
- Special characters need to be escaped, such as, 、"、\\, etc.

    - \\\\: escape backslashes.
    - \\,: escape commas. The commas separating parameters do not need to be escaped.
    - \\": escape double quotes, indicating that the double quotes of string parameters do not need to be escaped.


