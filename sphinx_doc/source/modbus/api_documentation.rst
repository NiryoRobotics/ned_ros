Use the Modbus TCP server
====================================

.. image:: ../../images/modbus_logo.jpg
         :alt: Modbus logo
         :width: 400px
         :align: center

In this document, we will focus on the Modbus/TCP server.

Ned is permanently running a Modbus TCP Server that enables Ned to communicate with a PLC, or another computer in the same network.

The Modbus/TCP server is running on **port 5020** by default.
It has been built on top of the `pymodbus <https://pymodbus.readthedocs.io/en/latest/index.html>`_ library.
This enables you to make Ned communicates with a PLC, or another computer on the same network.

Introduction
------------

All 4 Modbus datastores are implemented: :ref:`Coils<source/modbus/api_documentation:Coils>`, :ref:`Discrete inputs<source/modbus/api_documentation:Discrete inputs>`, :ref:`Holding registers<source/modbus/api_documentation:Holding registers>`, :ref:`Input registers<source/modbus/api_documentation:Input registers>`.
Each datastore has a different set of functionalities. Note that each datastore contains a completely different set of data.

Discrete Input and Input register are **READ-ONLY** tables. Those have been used to keep the robot state.

Coil and Holding Register are **READ/WRITE** tables. Those have been used to give user commands to the robot.
Hence, those 2 tables do not contain the robot state, but the last given command.

Address tables start at 0.


Coils
-------------------------------

+---------+------+----------------------+------------------------------------------------------------------------------------------------------------------------------+
| Coils   |      |                      |                                                                                                                              |
+=========+======+======================+==============================================================================================================================+
| Address | Type | Name                 | Description                                                                                                                  |
| 0       | bool | Digital Output 1     | Control the digital output value (0 = LOW 1 = HIGH)                                                                          |
| 1       | bool | Digital Output 2     |                                                                                                                              |
| 2       | bool | Digital Output 3     |                                                                                                                              |
| 3       | bool | Digital Output 4     |                                                                                                                              |
| 4       | bool | RESERVED             |                                                                                                                              |
| 5       | bool | RESERVED             |                                                                                                                              |
| 6       | bool | RESERVED             |                                                                                                                              |
| 7       | bool | RESERVED             |                                                                                                                              |
| 8       | bool | RESERVED             |                                                                                                                              |
| 9       | bool | RESERVED             |                                                                                                                              |
| 10      | bool | RESERVED             |                                                                                                                              |
| 11      | bool | RESERVED             |                                                                                                                              |
| 12      | bool | RESERVED             |                                                                                                                              |
| 13      | bool | RESERVED             |                                                                                                                              |
| 14      | bool | RESERVED             |                                                                                                                              |
| 15      | bool | RESERVED             |                                                                                                                              |
| 16      | bool | RESERVED             |                                                                                                                              |
| 17      | bool | RESERVED             |                                                                                                                              |
| 18      | bool | RESERVED             |                                                                                                                              |
| 19      | bool | RESERVED             |                                                                                                                              |
| 20      | bool | RESERVED             |                                                                                                                              |
| 21      | bool | RESERVED             |                                                                                                                              |
| 22      | bool | RESERVED             |                                                                                                                              |
| 23      | bool | RESERVED             |                                                                                                                              |
| 24      | bool | RESERVED             |                                                                                                                              |
| 25      | bool | RESERVED             |                                                                                                                              |
| 26      | bool | RESERVED             |                                                                                                                              |
| 27      | bool | RESERVED             |                                                                                                                              |
| 28      | bool | RESERVED             |                                                                                                                              |
| 29      | bool | RESERVED             |                                                                                                                              |
| 30      | bool | RESERVED             |                                                                                                                              |
| 31      | bool | RESERVED             |                                                                                                                              |
| 32      | bool | RESERVED             |                                                                                                                              |
| 33      | bool | RESERVED             |                                                                                                                              |
| 34      | bool | RESERVED             |                                                                                                                              |
| 35      | bool | RESERVED             |                                                                                                                              |
| 36      | bool | RESERVED             |                                                                                                                              |
| 37      | bool | RESERVED             |                                                                                                                              |
| 38      | bool | RESERVED             |                                                                                                                              |
| 39      | bool | RESERVED             |                                                                                                                              |
| 40      | bool | RESERVED             |                                                                                                                              |
| 41      | bool | RESERVED             |                                                                                                                              |
| 42      | bool | RESERVED             |                                                                                                                              |
| 43      | bool | RESERVED             |                                                                                                                              |
| 44      | bool | RESERVED             |                                                                                                                              |
| 45      | bool | RESERVED             |                                                                                                                              |
| 46      | bool | RESERVED             |                                                                                                                              |
| 47      | bool | RESERVED             |                                                                                                                              |
| 48      | bool | RESERVED             |                                                                                                                              |
| 49      | bool | RESERVED             |                                                                                                                              |
| 50      | bool | Tool Equipped        | is tool equipped                                                                                                             |
| 51      | bool | Tool Actuation       | is tool actuated                                                                                                             |
| 52      | bool | TCP Enabled          | is TCP transform enabled                                                                                                     |
| 53      | bool | Conveyor 1 Attached  | is conveyor attached writing True run a scan                                                                                 |
| 54      | bool | Conveyor 2 Attached  |                                                                                                                              |
| 55      | bool | RESERVED             |                                                                                                                              |
| 56      | bool | RESERVED             |                                                                                                                              |
| 57      | bool | RESERVED             |                                                                                                                              |
| 58      | bool | RESERVED             |                                                                                                                              |
| 59      | bool | RESERVED             |                                                                                                                              |
| 60      | bool | RESERVED             |                                                                                                                              |
| 61      | bool | RESERVED             |                                                                                                                              |
| 62      | bool | RESERVED             |                                                                                                                              |
| 63      | bool | RESERVED             |                                                                                                                              |
| 64      | bool | RESERVED             |                                                                                                                              |
| 65      | bool | RESERVED             |                                                                                                                              |
| 66      | bool | RESERVED             |                                                                                                                              |
| 67      | bool | RESERVED             |                                                                                                                              |
| 68      | bool | RESERVED             |                                                                                                                              |
| 69      | bool | RESERVED             |                                                                                                                              |
| 70      | bool | RESERVED             |                                                                                                                              |
| 71      | bool | RESERVED             |                                                                                                                              |
| 72      | bool | RESERVED             |                                                                                                                              |
| 73      | bool | Conveyor 1 Running   |                                                                                                                              |
| 74      | bool | Conveyor 2 Running   |                                                                                                                              |
| 75      | bool | RESERVED             |                                                                                                                              |
| 76      | bool | RESERVED             |                                                                                                                              |
| 77      | bool | RESERVED             |                                                                                                                              |
| 78      | bool | RESERVED             |                                                                                                                              |
| 79      | bool | RESERVED             |                                                                                                                              |
| 80      | bool | RESERVED             |                                                                                                                              |
| 81      | bool | RESERVED             |                                                                                                                              |
| 82      | bool | RESERVED             |                                                                                                                              |
| 83      | bool | RESERVED             |                                                                                                                              |
| 84      | bool | RESERVED             |                                                                                                                              |
| 85      | bool | RESERVED             |                                                                                                                              |
| 86      | bool | RESERVED             |                                                                                                                              |
| 87      | bool | RESERVED             |                                                                                                                              |
| 88      | bool | RESERVED             |                                                                                                                              |
| 89      | bool | RESERVED             |                                                                                                                              |
| 90      | bool | RESERVED             |                                                                                                                              |
| 91      | bool | RESERVED             |                                                                                                                              |
| 92      | bool | RESERVED             |                                                                                                                              |
| 93      | bool | Conveyor 1 Direction | (0 = BACKWARD 1 = FORWARD)                                                                                                   |
| 94      | bool | Conveyor 2 Direction |                                                                                                                              |
| 95      | bool | RESERVED             |                                                                                                                              |
| 96      | bool | RESERVED             |                                                                                                                              |
| 97      | bool | RESERVED             |                                                                                                                              |
| 98      | bool | RESERVED             |                                                                                                                              |
| 99      | bool | RESERVED             |                                                                                                                              |
| 100     | bool | RESERVED             |                                                                                                                              |
| 101     | bool | RESERVED             |                                                                                                                              |
| 102     | bool | RESERVED             |                                                                                                                              |
| 103     | bool | RESERVED             |                                                                                                                              |
| 104     | bool | RESERVED             |                                                                                                                              |
| 105     | bool | RESERVED             |                                                                                                                              |
| 106     | bool | RESERVED             |                                                                                                                              |
| 107     | bool | RESERVED             |                                                                                                                              |
| 108     | bool | RESERVED             |                                                                                                                              |
| 109     | bool | RESERVED             |                                                                                                                              |
| 110     | bool | RESERVED             |                                                                                                                              |
| 111     | bool | RESERVED             |                                                                                                                              |
| 112     | bool | RESERVED             |                                                                                                                              |
| 113     | bool | Robot Moving         | is the robot in movement. Writing True will result in the robot moving following the corresponding move type and move target |
| 114     | bool | Learning Mode        | is the robot in learning mode / freedrive                                                                                    |
| 115     | bool | Calibration Needed   | is calibration needed                                                                                                        |
| 116     | bool | Calibration          | is the calibration running                                                                                                   |
| 117     | bool | Collision Detected   | is there a collision detected                                                                                                |
| 118     | bool | RESERVED             |                                                                                                                              |
| 119     | bool | RESERVED             |                                                                                                                              |
| 120     | bool | RESERVED             |                                                                                                                              |
| 121     | bool | RESERVED             |                                                                                                                              |
| 122     | bool | RESERVED             |                                                                                                                              |
| 123     | bool | RESERVED             |                                                                                                                              |
| 124     | bool | RESERVED             |                                                                                                                              |
| 125     | bool | RESERVED             |                                                                                                                              |
| 126     | bool | RESERVED             |                                                                                                                              |
| 127     | bool | RESERVED             |                                                                                                                              |
| 128     | bool | RESERVED             |                                                                                                                              |
| 129     | bool | RESERVED             |                                                                                                                              |
| 130     | bool | RESERVED             |                                                                                                                              |
| 131     | bool | RESERVED             |                                                                                                                              |
| 132     | bool | RESERVED             |                                                                                                                              |
| 133     | bool | RESERVED             |                                                                                                                              |
| 134     | bool | RESERVED             |                                                                                                                              |
| 135     | bool | RESERVED             |                                                                                                                              |
| 136     | bool | RESERVED             |                                                                                                                              |
| 137     | bool | RESERVED             |                                                                                                                              |
| 138     | bool | RESERVED             |                                                                                                                              |
| 139     | bool | RESERVED             |                                                                                                                              |
| 140     | bool | RESERVED             |                                                                                                                              |
| 141     | bool | RESERVED             |                                                                                                                              |
| 142     | bool | RESERVED             |                                                                                                                              |
| 143     | bool | RESERVED             |                                                                                                                              |
| 144     | bool | RESERVED             |                                                                                                                              |
| 145     | bool | RESERVED             |                                                                                                                              |
| 146     | bool | RESERVED             |                                                                                                                              |
| 147     | bool | RESERVED             |                                                                                                                              |
| 148     | bool | RESERVED             |                                                                                                                              |
| 149     | bool | RESERVED             |                                                                                                                              |
| 150     | bool | RESERVED             |                                                                                                                              |
| 151     | bool | RESERVED             |                                                                                                                              |
| 152     | bool | RESERVED             |                                                                                                                              |
| 153     | bool | RESERVED             |                                                                                                                              |
| 154     | bool | RESERVED             |                                                                                                                              |
| 155     | bool | RESERVED             |                                                                                                                              |
| 156     | bool | RESERVED             |                                                                                                                              |
| 157     | bool | RESERVED             |                                                                                                                              |
| 158     | bool | RESERVED             |                                                                                                                              |
| 159     | bool | RESERVED             |                                                                                                                              |
| 160     | bool | RESERVED             |                                                                                                                              |
| 161     | bool | RESERVED             |                                                                                                                              |
| 162     | bool | RESERVED             |                                                                                                                              |
| 163     | bool | RESERVED             |                                                                                                                              |
| 164     | bool | RESERVED             |                                                                                                                              |
| 165     | bool | RESERVED             |                                                                                                                              |
| 166     | bool | RESERVED             |                                                                                                                              |
| 167     | bool | RESERVED             |                                                                                                                              |
| 168     | bool | RESERVED             |                                                                                                                              |
| 169     | bool | RESERVED             |                                                                                                                              |
| 170     | bool | RESERVED             |                                                                                                                              |
| 171     | bool | RESERVED             |                                                                                                                              |
| 172     | bool | RESERVED             |                                                                                                                              |
| 173     | bool | RESERVED             |                                                                                                                              |
| 174     | bool | RESERVED             |                                                                                                                              |
| 175     | bool | RESERVED             |                                                                                                                              |
| 176     | bool | RESERVED             |                                                                                                                              |
| 177     | bool | RESERVED             |                                                                                                                              |
| 178     | bool | RESERVED             |                                                                                                                              |
| 179     | bool | RESERVED             |                                                                                                                              |
| 180     | bool | RESERVED             |                                                                                                                              |
| 181     | bool | RESERVED             |                                                                                                                              |
| 182     | bool | RESERVED             |                                                                                                                              |
| 183     | bool | RESERVED             |                                                                                                                              |
| 184     | bool | RESERVED             |                                                                                                                              |
| 185     | bool | RESERVED             |                                                                                                                              |
| 186     | bool | RESERVED             |                                                                                                                              |
| 187     | bool | RESERVED             |                                                                                                                              |
| 188     | bool | RESERVED             |                                                                                                                              |
| 189     | bool | RESERVED             |                                                                                                                              |
| 190     | bool | RESERVED             |                                                                                                                              |
| 191     | bool | RESERVED             |                                                                                                                              |
| 192     | bool | RESERVED             |                                                                                                                              |
| 193     | bool | RESERVED             |                                                                                                                              |
| 194     | bool | RESERVED             |                                                                                                                              |
| 195     | bool | RESERVED             |                                                                                                                              |
| 196     | bool | RESERVED             |                                                                                                                              |
| 197     | bool | RESERVED             |                                                                                                                              |
| 198     | bool | RESERVED             |                                                                                                                              |
| 199     | bool | RESERVED             |                                                                                                                              |
| 200     | bool | Coil User Store 1    | Custom store for the user                                                                                                    |
| 201     | bool | Coil User Store 2    |                                                                                                                              |
| 202     | bool | Coil User Store 3    |                                                                                                                              |
| 203     | bool | Coil User Store 4    |                                                                                                                              |
| 204     | bool | Coil User Store 5    |                                                                                                                              |
| 205     | bool | Coil User Store 6    |                                                                                                                              |
| 206     | bool | Coil User Store 7    |                                                                                                                              |
| 207     | bool | Coil User Store 8    |                                                                                                                              |
| 208     | bool | Coil User Store 9    |                                                                                                                              |
| 209     | bool | Coil User Store 10   |                                                                                                                              |
| 210     | bool | Coil User Store 11   |                                                                                                                              |
| 211     | bool | Coil User Store 12   |                                                                                                                              |
| 212     | bool | Coil User Store 13   |                                                                                                                              |
| 213     | bool | Coil User Store 14   |                                                                                                                              |
| 214     | bool | Coil User Store 15   |                                                                                                                              |
| 215     | bool | Coil User Store 16   |                                                                                                                              |
| 216     | bool | Coil User Store 17   |                                                                                                                              |
| 217     | bool | Coil User Store 18   |                                                                                                                              |
| 218     | bool | Coil User Store 19   |                                                                                                                              |
| 219     | bool | Coil User Store 20   |                                                                                                                              |
| 220     | bool | Coil User Store 21   |                                                                                                                              |
| 221     | bool | Coil User Store 22   |                                                                                                                              |
| 222     | bool | Coil User Store 23   |                                                                                                                              |
| 223     | bool | Coil User Store 24   |                                                                                                                              |
| 224     | bool | Coil User Store 25   |                                                                                                                              |
| 225     | bool | Coil User Store 26   |                                                                                                                              |
| 226     | bool | Coil User Store 27   |                                                                                                                              |
| 227     | bool | Coil User Store 28   |                                                                                                                              |
| 228     | bool | Coil User Store 29   |                                                                                                                              |
| 229     | bool | Coil User Store 30   |                                                                                                                              |
| 230     | bool | Coil User Store 31   |                                                                                                                              |
| 231     | bool | Coil User Store 32   |                                                                                                                              |
| 232     | bool | Coil User Store 33   |                                                                                                                              |
| 233     | bool | Coil User Store 34   |                                                                                                                              |
| 234     | bool | Coil User Store 35   |                                                                                                                              |
| 235     | bool | Coil User Store 36   |                                                                                                                              |
| 236     | bool | Coil User Store 37   |                                                                                                                              |
| 237     | bool | Coil User Store 38   |                                                                                                                              |
| 238     | bool | Coil User Store 39   |                                                                                                                              |
| 239     | bool | Coil User Store 40   |                                                                                                                              |
| 240     | bool | Coil User Store 41   |                                                                                                                              |
| 241     | bool | Coil User Store 42   |                                                                                                                              |
| 242     | bool | Coil User Store 43   |                                                                                                                              |
| 243     | bool | Coil User Store 44   |                                                                                                                              |
| 244     | bool | Coil User Store 45   |                                                                                                                              |
| 245     | bool | Coil User Store 46   |                                                                                                                              |
| 246     | bool | Coil User Store 47   |                                                                                                                              |
| 247     | bool | Coil User Store 48   |                                                                                                                              |
| 248     | bool | Coil User Store 49   |                                                                                                                              |
| 249     | bool | Coil User Store 50   |                                                                                                                              |
| 250     | bool | Coil User Store 51   |                                                                                                                              |
| 251     | bool | Coil User Store 52   |                                                                                                                              |
| 252     | bool | Coil User Store 53   |                                                                                                                              |
| 253     | bool | Coil User Store 54   |                                                                                                                              |
| 254     | bool | Coil User Store 55   |                                                                                                                              |
| 255     | bool | Coil User Store 56   |                                                                                                                              |
| 256     | bool | Coil User Store 57   |                                                                                                                              |
| 257     | bool | Coil User Store 58   |                                                                                                                              |
| 258     | bool | Coil User Store 59   |                                                                                                                              |
| 259     | bool | Coil User Store 60   |                                                                                                                              |
| 260     | bool | Coil User Store 61   |                                                                                                                              |
| 261     | bool | Coil User Store 62   |                                                                                                                              |
| 262     | bool | Coil User Store 63   |                                                                                                                              |
| 263     | bool | Coil User Store 64   |                                                                                                                              |
| 264     | bool | Coil User Store 65   |                                                                                                                              |
| 265     | bool | Coil User Store 66   |                                                                                                                              |
| 266     | bool | Coil User Store 67   |                                                                                                                              |
| 267     | bool | Coil User Store 68   |                                                                                                                              |
| 268     | bool | Coil User Store 69   |                                                                                                                              |
| 269     | bool | Coil User Store 70   |                                                                                                                              |
| 270     | bool | Coil User Store 71   |                                                                                                                              |
| 271     | bool | Coil User Store 72   |                                                                                                                              |
| 272     | bool | Coil User Store 73   |                                                                                                                              |
| 273     | bool | Coil User Store 74   |                                                                                                                              |
| 274     | bool | Coil User Store 75   |                                                                                                                              |
| 275     | bool | Coil User Store 76   |                                                                                                                              |
| 276     | bool | Coil User Store 77   |                                                                                                                              |
| 277     | bool | Coil User Store 78   |                                                                                                                              |
| 278     | bool | Coil User Store 79   |                                                                                                                              |
| 279     | bool | Coil User Store 80   |                                                                                                                              |
| 280     | bool | Coil User Store 81   |                                                                                                                              |
| 281     | bool | Coil User Store 82   |                                                                                                                              |
| 282     | bool | Coil User Store 83   |                                                                                                                              |
| 283     | bool | Coil User Store 84   |                                                                                                                              |
| 284     | bool | Coil User Store 85   |                                                                                                                              |
| 285     | bool | Coil User Store 86   |                                                                                                                              |
| 286     | bool | Coil User Store 87   |                                                                                                                              |
| 287     | bool | Coil User Store 88   |                                                                                                                              |
| 288     | bool | Coil User Store 89   |                                                                                                                              |
| 289     | bool | Coil User Store 90   |                                                                                                                              |
| 290     | bool | Coil User Store 91   |                                                                                                                              |
| 291     | bool | Coil User Store 92   |                                                                                                                              |
| 292     | bool | Coil User Store 93   |                                                                                                                              |
| 293     | bool | Coil User Store 94   |                                                                                                                              |
| 294     | bool | Coil User Store 95   |                                                                                                                              |
| 295     | bool | Coil User Store 96   |                                                                                                                              |
| 296     | bool | Coil User Store 97   |                                                                                                                              |
| 297     | bool | Coil User Store 98   |                                                                                                                              |
| 298     | bool | Coil User Store 99   |                                                                                                                              |
| 299     | bool | Coil User Store 100  |                                                                                                                              |
+---------+------+----------------------+------------------------------------------------------------------------------------------------------------------------------+


Discrete inputs
-------------------------------

+-----------------+------+---------------------+-------------------------------------------------+
| Discrete Inputs |      |                     |                                                 |
+=================+======+=====================+=================================================+
| Address         | Type | Name                | Description                                     |
| 0               | bool | Digital Input 1     | Get the digital input value (0 = LOW, 1 = HIGH) |
| 1               | bool | Digital Input 2     |                                                 |
| 2               | bool | Digital Input 3     |                                                 |
| 3               | bool | Digital Input 4     |                                                 |
| 4               | bool | RESERVED            |                                                 |
| 5               | bool | RESERVED            |                                                 |
| 6               | bool | RESERVED            |                                                 |
| 7               | bool | RESERVED            |                                                 |
| 8               | bool | RESERVED            |                                                 |
| 9               | bool | RESERVED            |                                                 |
| 10              | bool | RESERVED            |                                                 |
| 11              | bool | RESERVED            |                                                 |
| 12              | bool | RESERVED            |                                                 |
| 13              | bool | RESERVED            |                                                 |
| 14              | bool | RESERVED            |                                                 |
| 15              | bool | RESERVED            |                                                 |
| 16              | bool | RESERVED            |                                                 |
| 17              | bool | RESERVED            |                                                 |
| 18              | bool | RESERVED            |                                                 |
| 19              | bool | RESERVED            |                                                 |
| 20              | bool | RESERVED            |                                                 |
| 21              | bool | RESERVED            |                                                 |
| 22              | bool | RESERVED            |                                                 |
| 23              | bool | RESERVED            |                                                 |
| 24              | bool | RESERVED            |                                                 |
| 25              | bool | RESERVED            |                                                 |
| 26              | bool | RESERVED            |                                                 |
| 27              | bool | RESERVED            |                                                 |
| 28              | bool | RESERVED            |                                                 |
| 29              | bool | RESERVED            |                                                 |
| 30              | bool | RESERVED            |                                                 |
| 31              | bool | RESERVED            |                                                 |
| 32              | bool | RESERVED            |                                                 |
| 33              | bool | RESERVED            |                                                 |
| 34              | bool | RESERVED            |                                                 |
| 35              | bool | RESERVED            |                                                 |
| 36              | bool | RESERVED            |                                                 |
| 37              | bool | RESERVED            |                                                 |
| 38              | bool | RESERVED            |                                                 |
| 39              | bool | RESERVED            |                                                 |
| 40              | bool | RESERVED            |                                                 |
| 41              | bool | RESERVED            |                                                 |
| 42              | bool | RESERVED            |                                                 |
| 43              | bool | RESERVED            |                                                 |
| 44              | bool | RESERVED            |                                                 |
| 45              | bool | RESERVED            |                                                 |
| 46              | bool | RESERVED            |                                                 |
| 47              | bool | RESERVED            |                                                 |
| 48              | bool | RESERVED            |                                                 |
| 49              | bool | RESERVED            |                                                 |
| 50              | bool | Motor Connection    | is motor connection OK                          |
| 51              | bool | Executing Command   | is there a command running                      |
| 52              | bool | Vision Target Found | is a vision target found                        |
+-----------------+------+---------------------+-------------------------------------------------+



Holding registers
-------------------------------

+-------------------+----------+---------------------------+-------------------------------------------------------------------------+
| Holding Registers |          |                           |                                                                         |
+===================+==========+===========================+=========================================================================+
| Address           | Type     | Name                      | Description                                                             |
| 0                 | float    | Analog Output 1           | Control the analog output value (V)                                     |
| 1                 |          |                           |                                                                         |
| 2                 | float    | Analog Output 2           |                                                                         |
| 3                 |          |                           |                                                                         |
| 4                 | RESERVED |                           |                                                                         |
| 5                 | RESERVED |                           |                                                                         |
| 6                 | RESERVED |                           |                                                                         |
| 7                 | RESERVED |                           |                                                                         |
| 8                 | RESERVED |                           |                                                                         |
| 9                 | RESERVED |                           |                                                                         |
| 10                | RESERVED |                           |                                                                         |
| 11                | RESERVED |                           |                                                                         |
| 12                | RESERVED |                           |                                                                         |
| 13                | RESERVED |                           |                                                                         |
| 14                | RESERVED |                           |                                                                         |
| 15                | RESERVED |                           |                                                                         |
| 16                | RESERVED |                           |                                                                         |
| 17                | RESERVED |                           |                                                                         |
| 18                | RESERVED |                           |                                                                         |
| 19                | RESERVED |                           |                                                                         |
| 20                | RESERVED |                           |                                                                         |
| 21                | RESERVED |                           |                                                                         |
| 22                | RESERVED |                           |                                                                         |
| 23                | RESERVED |                           |                                                                         |
| 24                | RESERVED |                           |                                                                         |
| 25                | RESERVED |                           |                                                                         |
| 26                | RESERVED |                           |                                                                         |
| 27                | RESERVED |                           |                                                                         |
| 28                | RESERVED |                           |                                                                         |
| 29                | RESERVED |                           |                                                                         |
| 30                | RESERVED |                           |                                                                         |
| 31                | RESERVED |                           |                                                                         |
| 32                | RESERVED |                           |                                                                         |
| 33                | RESERVED |                           |                                                                         |
| 34                | RESERVED |                           |                                                                         |
| 35                | RESERVED |                           |                                                                         |
| 36                | RESERVED |                           |                                                                         |
| 37                | RESERVED |                           |                                                                         |
| 38                | RESERVED |                           |                                                                         |
| 39                | RESERVED |                           |                                                                         |
| 40                | RESERVED |                           |                                                                         |
| 41                | RESERVED |                           |                                                                         |
| 42                | RESERVED |                           |                                                                         |
| 43                | RESERVED |                           |                                                                         |
| 44                | RESERVED |                           |                                                                         |
| 45                | RESERVED |                           |                                                                         |
| 46                | RESERVED |                           |                                                                         |
| 47                | RESERVED |                           |                                                                         |
| 48                | RESERVED |                           |                                                                         |
| 49                | RESERVED |                           |                                                                         |
| 50                | float    | Joint 1 Target            | Move joint target position (rad)                                        |
| 51                |          |                           |                                                                         |
| 52                | float    | Joint 2 Target            |                                                                         |
| 53                |          |                           |                                                                         |
| 54                | float    | Joint 3 Target            |                                                                         |
| 55                |          |                           |                                                                         |
| 56                | float    | Joint 4 Target            |                                                                         |
| 57                |          |                           |                                                                         |
| 58                | float    | Joint 5 Target            |                                                                         |
| 59                |          |                           |                                                                         |
| 60                | float    | Joint 6 Target            |                                                                         |
| 61                |          |                           |                                                                         |
| 62                | float    | Pose Target X             | Move pose target position (m)                                           |
| 63                |          |                           |                                                                         |
| 64                | float    | Pose Target Y             |                                                                         |
| 65                |          |                           |                                                                         |
| 66                | float    | Pose Target Z             |                                                                         |
| 67                |          |                           |                                                                         |
| 68                | float    | Pose Target Roll          | Move pose target orientation (rad)                                      |
| 69                |          |                           |                                                                         |
| 70                | float    | Pose Target Pitch         |                                                                         |
| 71                |          |                           |                                                                         |
| 72                | float    | Pose Target Yaw           |                                                                         |
| 73                |          |                           |                                                                         |
| 74                | int      | Move Type                 | (0 = MOVE_JOINT, 1 = MOVE_POSE, 2 = MOVE_LINEAR)                        |
| 75                | float    | TCP Transformation X      | TCP translation (m)                                                     |
| 76                |          |                           |                                                                         |
| 77                | float    | TCP Transformation Y      |                                                                         |
| 78                |          |                           |                                                                         |
| 79                | float    | TCP Transformation Z      |                                                                         |
| 80                |          |                           |                                                                         |
| 81                | float    | TCP Transformation Roll   | TCP rotation (rad)                                                      |
| 82                |          |                           |                                                                         |
| 83                | float    | TCP Transformation Pitch  |                                                                         |
| 84                |          |                           |                                                                         |
| 85                | float    | TCP Transformation Yaw    |                                                                         |
| 86                |          |                           |                                                                         |
| 87                | int      | Conveyor 1 Speed          | percentage                                                              |
| 88                | int      | Conveyor 2 Speed          |                                                                         |
| 89                | RESERVED |                           |                                                                         |
| 90                | RESERVED |                           |                                                                         |
| 91                | RESERVED |                           |                                                                         |
| 92                | RESERVED |                           |                                                                         |
| 93                | RESERVED |                           |                                                                         |
| 94                | RESERVED |                           |                                                                         |
| 95                | RESERVED |                           |                                                                         |
| 96                | RESERVED |                           |                                                                         |
| 97                | RESERVED |                           |                                                                         |
| 98                | RESERVED |                           |                                                                         |
| 99                | RESERVED |                           |                                                                         |
| 100               | RESERVED |                           |                                                                         |
| 101               | RESERVED |                           |                                                                         |
| 102               | RESERVED |                           |                                                                         |
| 103               | RESERVED |                           |                                                                         |
| 104               | RESERVED |                           |                                                                         |
| 105               | RESERVED |                           |                                                                         |
| 106               | RESERVED |                           |                                                                         |
| 107               | int      | Gripper Open Speed        | percentage                                                              |
| 108               | int      | Gripper Open Max Torque   | percentage                                                              |
| 109               | int      | Gripper Open Hold Torque  | percentage                                                              |
| 110               | int      | Gripper Close Speed       | percentage                                                              |
| 111               | int      | Gripper Close Max Torque  | percentage                                                              |
| 112               | int      | Gripper Close Hold Torque | percentage                                                              |
| 113               | float    | Relative X                | Relative position to the workspace                                      |
| 114               |          |                           |                                                                         |
| 115               | float    | Relative Y                |                                                                         |
| 116               |          |                           |                                                                         |
| 117               | float    | Relative Yaw              |                                                                         |
| 118               |          |                           |                                                                         |
| 119               | str      | Workspace Name            | Name of the workspace to use for all the vison related functions.       |
| 120               | str      |                           |                                                                         |
| 121               | str      |                           |                                                                         |
| 122               | str      |                           |                                                                         |
| 123               | str      |                           |                                                                         |
| 124               | str      |                           |                                                                         |
| 125               | str      |                           |                                                                         |
| 126               | str      |                           |                                                                         |
| 127               | str      |                           |                                                                         |
| 128               | str      |                           |                                                                         |
| 129               | str      |                           |                                                                         |
| 130               | str      |                           |                                                                         |
| 131               | str      |                           |                                                                         |
| 132               | str      |                           |                                                                         |
| 133               | str      |                           |                                                                         |
| 134               | str      |                           |                                                                         |
| 135               | str      |                           |                                                                         |
| 136               | str      |                           |                                                                         |
| 137               | str      |                           |                                                                         |
| 138               | str      |                           |                                                                         |
| 139               | int      | Height Offset             | height offset for the relative to absolute pose transforms calculations |
| 140               | int      | Target Shape              | (0 = ANY,1 = CIRCLE,2 = SQUARE)                                         |
| 141               | int      | Target Color              | (0 = ANY, 1 = RED, 2 = GREEN, 3 = BLUE)                                 |
| 142               | int      | Arm Speed                 | percentage                                                              |
| 143               | RESERVED |                           |                                                                         |
| 144               | RESERVED |                           |                                                                         |
| 145               | RESERVED |                           |                                                                         |
| 146               | RESERVED |                           |                                                                         |
| 147               | RESERVED |                           |                                                                         |
| 148               | RESERVED |                           |                                                                         |
| 149               | RESERVED |                           |                                                                         |
| 150               | RESERVED |                           |                                                                         |
| 151               | RESERVED |                           |                                                                         |
| 152               | RESERVED |                           |                                                                         |
| 153               | RESERVED |                           |                                                                         |
| 154               | RESERVED |                           |                                                                         |
| 155               | RESERVED |                           |                                                                         |
| 156               | RESERVED |                           |                                                                         |
| 157               | RESERVED |                           |                                                                         |
| 158               | RESERVED |                           |                                                                         |
| 159               | RESERVED |                           |                                                                         |
| 160               | RESERVED |                           |                                                                         |
| 161               | RESERVED |                           |                                                                         |
| 162               | RESERVED |                           |                                                                         |
| 163               | RESERVED |                           |                                                                         |
| 164               | RESERVED |                           |                                                                         |
| 165               | RESERVED |                           |                                                                         |
| 166               | RESERVED |                           |                                                                         |
| 167               | RESERVED |                           |                                                                         |
| 168               | RESERVED |                           |                                                                         |
| 169               | RESERVED |                           |                                                                         |
| 170               | RESERVED |                           |                                                                         |
| 171               | RESERVED |                           |                                                                         |
| 172               | RESERVED |                           |                                                                         |
| 173               | RESERVED |                           |                                                                         |
| 174               | RESERVED |                           |                                                                         |
| 175               | RESERVED |                           |                                                                         |
| 176               | RESERVED |                           |                                                                         |
| 177               | RESERVED |                           |                                                                         |
| 178               | RESERVED |                           |                                                                         |
| 179               | RESERVED |                           |                                                                         |
| 180               | RESERVED |                           |                                                                         |
| 181               | RESERVED |                           |                                                                         |
| 182               | RESERVED |                           |                                                                         |
| 183               | RESERVED |                           |                                                                         |
| 184               | RESERVED |                           |                                                                         |
| 185               | RESERVED |                           |                                                                         |
| 186               | RESERVED |                           |                                                                         |
| 187               | RESERVED |                           |                                                                         |
| 188               | RESERVED |                           |                                                                         |
| 189               | RESERVED |                           |                                                                         |
| 190               | RESERVED |                           |                                                                         |
| 191               | RESERVED |                           |                                                                         |
| 192               | RESERVED |                           |                                                                         |
| 193               | RESERVED |                           |                                                                         |
| 194               | RESERVED |                           |                                                                         |
| 195               | RESERVED |                           |                                                                         |
| 196               | RESERVED |                           |                                                                         |
| 197               | RESERVED |                           |                                                                         |
| 198               | RESERVED |                           |                                                                         |
| 199               | RESERVED |                           |                                                                         |
| 200               | float    | Float User Store 1        | Custom store for the user                                               |
| 201               |          |                           |                                                                         |
| 202               | float    | Float User Store 2        |                                                                         |
| 203               |          |                           |                                                                         |
| 204               | float    | Float User Store 3        |                                                                         |
| 205               |          |                           |                                                                         |
| 206               | float    | Float User Store 4        |                                                                         |
| 207               |          |                           |                                                                         |
| 208               | float    | Float User Store 5        |                                                                         |
| 209               |          |                           |                                                                         |
| 210               | float    | Float User Store 6        |                                                                         |
| 211               |          |                           |                                                                         |
| 212               | float    | Float User Store 7        |                                                                         |
| 213               |          |                           |                                                                         |
| 214               | float    | Float User Store 8        |                                                                         |
| 215               |          |                           |                                                                         |
| 216               | float    | Float User Store 9        |                                                                         |
| 217               |          |                           |                                                                         |
| 218               | float    | Float User Store 10       |                                                                         |
| 219               |          |                           |                                                                         |
| 220               | float    | Float User Store 11       |                                                                         |
| 221               |          |                           |                                                                         |
| 222               | float    | Float User Store 12       |                                                                         |
| 223               |          |                           |                                                                         |
| 224               | float    | Float User Store 13       |                                                                         |
| 225               |          |                           |                                                                         |
| 226               | float    | Float User Store 14       |                                                                         |
| 227               |          |                           |                                                                         |
| 228               | float    | Float User Store 15       |                                                                         |
| 229               |          |                           |                                                                         |
| 230               | float    | Float User Store 16       |                                                                         |
| 231               |          |                           |                                                                         |
| 232               | float    | Float User Store 17       |                                                                         |
| 233               |          |                           |                                                                         |
| 234               | float    | Float User Store 18       |                                                                         |
| 235               |          |                           |                                                                         |
| 236               | float    | Float User Store 19       |                                                                         |
| 237               |          |                           |                                                                         |
| 238               | float    | Float User Store 20       |                                                                         |
| 239               |          |                           |                                                                         |
| 240               | float    | Float User Store 21       |                                                                         |
| 241               |          |                           |                                                                         |
| 242               | float    | Float User Store 22       |                                                                         |
| 243               |          |                           |                                                                         |
| 244               | float    | Float User Store 23       |                                                                         |
| 245               |          |                           |                                                                         |
| 246               | float    | Float User Store 24       |                                                                         |
| 247               |          |                           |                                                                         |
| 248               | float    | Float User Store 25       |                                                                         |
| 249               |          |                           |                                                                         |
| 250               | float    | Float User Store 26       |                                                                         |
| 251               |          |                           |                                                                         |
| 252               | float    | Float User Store 27       |                                                                         |
| 253               |          |                           |                                                                         |
| 254               | float    | Float User Store 28       |                                                                         |
| 255               |          |                           |                                                                         |
| 256               | float    | Float User Store 29       |                                                                         |
| 257               |          |                           |                                                                         |
| 258               | float    | Float User Store 30       |                                                                         |
| 259               |          |                           |                                                                         |
| 260               | float    | Float User Store 31       |                                                                         |
| 261               |          |                           |                                                                         |
| 262               | float    | Float User Store 32       |                                                                         |
| 263               |          |                           |                                                                         |
| 264               | float    | Float User Store 33       |                                                                         |
| 265               |          |                           |                                                                         |
| 266               | float    | Float User Store 34       |                                                                         |
| 267               |          |                           |                                                                         |
| 268               | float    | Float User Store 35       |                                                                         |
| 269               |          |                           |                                                                         |
| 270               | float    | Float User Store 36       |                                                                         |
| 271               |          |                           |                                                                         |
| 272               | float    | Float User Store 37       |                                                                         |
| 273               |          |                           |                                                                         |
| 274               | float    | Float User Store 38       |                                                                         |
| 275               |          |                           |                                                                         |
| 276               | float    | Float User Store 39       |                                                                         |
| 277               |          |                           |                                                                         |
| 278               | float    | Float User Store 40       |                                                                         |
| 279               |          |                           |                                                                         |
| 280               | float    | Float User Store 41       |                                                                         |
| 281               |          |                           |                                                                         |
| 282               | float    | Float User Store 42       |                                                                         |
| 283               |          |                           |                                                                         |
| 284               | float    | Float User Store 43       |                                                                         |
| 285               |          |                           |                                                                         |
| 286               | float    | Float User Store 44       |                                                                         |
| 287               |          |                           |                                                                         |
| 288               | float    | Float User Store 45       |                                                                         |
| 289               |          |                           |                                                                         |
| 290               | float    | Float User Store 46       |                                                                         |
| 291               |          |                           |                                                                         |
| 292               | float    | Float User Store 47       |                                                                         |
| 293               |          |                           |                                                                         |
| 294               | float    | Float User Store 48       |                                                                         |
| 295               |          |                           |                                                                         |
| 296               | float    | Float User Store 49       |                                                                         |
| 297               |          |                           |                                                                         |
| 298               | float    | Float User Store 50       |                                                                         |
| 299               |          |                           |                                                                         |
| 300               | float    | Float User Store 51       |                                                                         |
| 301               |          |                           |                                                                         |
| 302               | float    | Float User Store 52       |                                                                         |
| 303               |          |                           |                                                                         |
| 304               | float    | Float User Store 53       |                                                                         |
| 305               |          |                           |                                                                         |
| 306               | float    | Float User Store 54       |                                                                         |
| 307               |          |                           |                                                                         |
| 308               | float    | Float User Store 55       |                                                                         |
| 309               |          |                           |                                                                         |
| 310               | float    | Float User Store 56       |                                                                         |
| 311               |          |                           |                                                                         |
| 312               | float    | Float User Store 57       |                                                                         |
| 313               |          |                           |                                                                         |
| 314               | float    | Float User Store 58       |                                                                         |
| 315               |          |                           |                                                                         |
| 316               | float    | Float User Store 59       |                                                                         |
| 317               |          |                           |                                                                         |
| 318               | float    | Float User Store 60       |                                                                         |
| 319               |          |                           |                                                                         |
| 320               | float    | Float User Store 61       |                                                                         |
| 321               |          |                           |                                                                         |
| 322               | float    | Float User Store 62       |                                                                         |
| 323               |          |                           |                                                                         |
| 324               | float    | Float User Store 63       |                                                                         |
| 325               |          |                           |                                                                         |
| 326               | float    | Float User Store 64       |                                                                         |
| 327               |          |                           |                                                                         |
| 328               | float    | Float User Store 65       |                                                                         |
| 329               |          |                           |                                                                         |
| 330               | float    | Float User Store 66       |                                                                         |
| 331               |          |                           |                                                                         |
| 332               | float    | Float User Store 67       |                                                                         |
| 333               |          |                           |                                                                         |
| 334               | float    | Float User Store 68       |                                                                         |
| 335               |          |                           |                                                                         |
| 336               | float    | Float User Store 69       |                                                                         |
| 337               |          |                           |                                                                         |
| 338               | float    | Float User Store 70       |                                                                         |
| 339               |          |                           |                                                                         |
| 340               | float    | Float User Store 71       |                                                                         |
| 341               |          |                           |                                                                         |
| 342               | float    | Float User Store 72       |                                                                         |
| 343               |          |                           |                                                                         |
| 344               | float    | Float User Store 73       |                                                                         |
| 345               |          |                           |                                                                         |
| 346               | float    | Float User Store 74       |                                                                         |
| 347               |          |                           |                                                                         |
| 348               | float    | Float User Store 75       |                                                                         |
| 349               |          |                           |                                                                         |
| 350               | float    | Float User Store 76       |                                                                         |
| 351               |          |                           |                                                                         |
| 352               | float    | Float User Store 77       |                                                                         |
| 353               |          |                           |                                                                         |
| 354               | float    | Float User Store 78       |                                                                         |
| 355               |          |                           |                                                                         |
| 356               | float    | Float User Store 79       |                                                                         |
| 357               |          |                           |                                                                         |
| 358               | float    | Float User Store 80       |                                                                         |
| 359               |          |                           |                                                                         |
| 360               | float    | Float User Store 81       |                                                                         |
| 361               |          |                           |                                                                         |
| 362               | float    | Float User Store 82       |                                                                         |
| 363               |          |                           |                                                                         |
| 364               | float    | Float User Store 83       |                                                                         |
| 365               |          |                           |                                                                         |
| 366               | float    | Float User Store 84       |                                                                         |
| 367               |          |                           |                                                                         |
| 368               | float    | Float User Store 85       |                                                                         |
| 369               |          |                           |                                                                         |
| 370               | float    | Float User Store 86       |                                                                         |
| 371               |          |                           |                                                                         |
| 372               | float    | Float User Store 87       |                                                                         |
| 373               |          |                           |                                                                         |
| 374               | float    | Float User Store 88       |                                                                         |
| 375               |          |                           |                                                                         |
| 376               | float    | Float User Store 89       |                                                                         |
| 377               |          |                           |                                                                         |
| 378               | float    | Float User Store 90       |                                                                         |
| 379               |          |                           |                                                                         |
| 380               | float    | Float User Store 91       |                                                                         |
| 381               |          |                           |                                                                         |
| 382               | float    | Float User Store 92       |                                                                         |
| 383               |          |                           |                                                                         |
| 384               | float    | Float User Store 93       |                                                                         |
| 385               |          |                           |                                                                         |
| 386               | float    | Float User Store 94       |                                                                         |
| 387               |          |                           |                                                                         |
| 388               | float    | Float User Store 95       |                                                                         |
| 389               |          |                           |                                                                         |
| 390               | float    | Float User Store 96       |                                                                         |
| 391               |          |                           |                                                                         |
| 392               | float    | Float User Store 97       |                                                                         |
| 393               |          |                           |                                                                         |
| 394               | float    | Float User Store 98       |                                                                         |
| 395               |          |                           |                                                                         |
| 396               | float    | Float User Store 99       |                                                                         |
| 397               |          |                           |                                                                         |
| 398               | float    | Float User Store 100      |                                                                         |
| 399               |          |                           |                                                                         |
+-------------------+----------+---------------------------+-------------------------------------------------------------------------+

Input registers
-------------------------------

+--------------------+----------+----------------------------------------+--------------------------------------------------------------------------------------+
| Input Registers    |          |                                        |                                                                                      |
+====================+==========+========================================+======================================================================================+
| Address            | Type     | Name                                   | Description                                                                          |
| 0                  | float    | Analog Input 1                         | Get the analog input value (V)                                                       |
| 1                  |          |                                        |                                                                                      |
| 2                  | float    | Analog Input 2                         |                                                                                      |
| 3                  |          |                                        |                                                                                      |
| 4                  | RESERVED |                                        |                                                                                      |
| 5                  | RESERVED |                                        |                                                                                      |
| 6                  | RESERVED |                                        |                                                                                      |
| 7                  | RESERVED |                                        |                                                                                      |
| 8                  | RESERVED |                                        |                                                                                      |
| 9                  | RESERVED |                                        |                                                                                      |
| 10                 | RESERVED |                                        |                                                                                      |
| 11                 | RESERVED |                                        |                                                                                      |
| 12                 | RESERVED |                                        |                                                                                      |
| 13                 | RESERVED |                                        |                                                                                      |
| 14                 | RESERVED |                                        |                                                                                      |
| 15                 | RESERVED |                                        |                                                                                      |
| 16                 | RESERVED |                                        |                                                                                      |
| 17                 | RESERVED |                                        |                                                                                      |
| 18                 | RESERVED |                                        |                                                                                      |
| 19                 | RESERVED |                                        |                                                                                      |
| 20                 | RESERVED |                                        |                                                                                      |
| 21                 | RESERVED |                                        |                                                                                      |
| 22                 | RESERVED |                                        |                                                                                      |
| 23                 | RESERVED |                                        |                                                                                      |
| 24                 | RESERVED |                                        |                                                                                      |
| 25                 | RESERVED |                                        |                                                                                      |
| 26                 | RESERVED |                                        |                                                                                      |
| 27                 | RESERVED |                                        |                                                                                      |
| 28                 | RESERVED |                                        |                                                                                      |
| 29                 | RESERVED |                                        |                                                                                      |
| 30                 | RESERVED |                                        |                                                                                      |
| 31                 | RESERVED |                                        |                                                                                      |
| 32                 | RESERVED |                                        |                                                                                      |
| 33                 | RESERVED |                                        |                                                                                      |
| 34                 | RESERVED |                                        |                                                                                      |
| 35                 | RESERVED |                                        |                                                                                      |
| 36                 | RESERVED |                                        |                                                                                      |
| 37                 | RESERVED |                                        |                                                                                      |
| 38                 | RESERVED |                                        |                                                                                      |
| 39                 | RESERVED |                                        |                                                                                      |
| 40                 | RESERVED |                                        |                                                                                      |
| 41                 | RESERVED |                                        |                                                                                      |
| 42                 | RESERVED |                                        |                                                                                      |
| 43                 | RESERVED |                                        |                                                                                      |
| 44                 | RESERVED |                                        |                                                                                      |
| 45                 | RESERVED |                                        |                                                                                      |
| 46                 | RESERVED |                                        |                                                                                      |
| 47                 | RESERVED |                                        |                                                                                      |
| 48                 | RESERVED |                                        |                                                                                      |
| 49                 | RESERVED |                                        |                                                                                      |
| 50                 | float    | Current Joint 1 State                  | Current joints position (rad)                                                        |
| 51                 |          |                                        |                                                                                      |
| 52                 | float    | Current Joint 2 State                  |                                                                                      |
| 53                 |          |                                        |                                                                                      |
| 54                 | float    | Current Joint 3 State                  |                                                                                      |
| 55                 |          |                                        |                                                                                      |
| 56                 | float    | Current Joint 4 State                  |                                                                                      |
| 57                 |          |                                        |                                                                                      |
| 58                 | float    | Current Joint 5 State                  |                                                                                      |
| 59                 |          |                                        |                                                                                      |
| 60                 | float    | Current Joint 6 State                  |                                                                                      |
| 61                 |          |                                        |                                                                                      |
| 62                 | float    | Current Pose State X                   | Current pose position (m)                                                            |
| 63                 |          |                                        |                                                                                      |
| 64                 | float    | Current Pose State Y                   |                                                                                      |
| 65                 |          |                                        |                                                                                      |
| 66                 | float    | Current Pose State Z                   |                                                                                      |
| 67                 |          |                                        |                                                                                      |
| 68                 | float    | Current Pose State Roll                | Current pose orientation (rad)                                                       |
| 69                 |          |                                        |                                                                                      |
| 70                 | float    | Current Pose State Pitch               |                                                                                      |
| 71                 |          |                                        |                                                                                      |
| 72                 | float    | Current Pose State Yaw                 |                                                                                      |
| 73                 |          |                                        |                                                                                      |
| 74                 | float    | Absolute Pose From Relative Pose X     | "Absolute position calculated from relative x, relative y and relative yaw (m)"      |
| 75                 |          |                                        |                                                                                      |
| 76                 | float    | Absolute Pose From Relative Pose Y     |                                                                                      |
| 77                 |          |                                        |                                                                                      |
| 78                 | float    | Absolute Pose From Relative Pose Z     |                                                                                      |
| 79                 |          |                                        |                                                                                      |
| 80                 | float    | Absolute Pose From Relative Pose Roll  | "Absolute orientation calculated from relative x, relative y and relative yaw (rad)" |
| 81                 |          |                                        |                                                                                      |
| 82                 | float    | Absolute Pose From Relative Pose Pitch |                                                                                      |
| 83                 |          |                                        |                                                                                      |
| 84                 | float    | Absolute Pose From Relative Pose Yaw   |                                                                                      |
| 85                 |          |                                        |                                                                                      |
| 86                 | float    | Vision Target Pose X                   | Position of the vision target (m)                                                    |
| 87                 |          |                                        |                                                                                      |
| 88                 | float    | Vision Target Pose Y                   |                                                                                      |
| 89                 |          |                                        |                                                                                      |
| 90                 | float    | Vision Target Pose Z                   |                                                                                      |
| 91                 |          |                                        |                                                                                      |
| 92                 | float    | Vision Target Pose Roll                | Orientation of the vision target (rad)                                               |
| 93                 |          |                                        |                                                                                      |
| 94                 | float    | Vision Target Pose Pitch               |                                                                                      |
| 95                 |          |                                        |                                                                                      |
| 96                 | float    | Vision Target Pose Yaw                 |                                                                                      |
| 97                 |          |                                        |                                                                                      |
| 98                 | int      | Vision Target Shape                    | 0 = ANY, 1 = CIRCLE, 2 = SQUARE                                                      |
| 99                 | int      | Vision Target Color                    | (0 = ANY, 1 = RED, 2 = GREEN, 3 = BLUE)                                              |
| 100                | int      | Conveyor 1 ID                          |                                                                                      |
| 101                | int      | Conveyor 2 ID                          |                                                                                      |
| 102                | RESERVED |                                        |                                                                                      |
| 103                | RESERVED |                                        |                                                                                      |
| 104                | RESERVED |                                        |                                                                                      |
| 105                | RESERVED |                                        |                                                                                      |
| 106                | RESERVED |                                        |                                                                                      |
| 107                | RESERVED |                                        |                                                                                      |
| 108                | RESERVED |                                        |                                                                                      |
| 109                | RESERVED |                                        |                                                                                      |
| 110                | RESERVED |                                        |                                                                                      |
| 111                | RESERVED |                                        |                                                                                      |
| 112                | RESERVED |                                        |                                                                                      |
| 113                | RESERVED |                                        |                                                                                      |
| 114                | RESERVED |                                        |                                                                                      |
| 115                | RESERVED |                                        |                                                                                      |
| 116                | RESERVED |                                        |                                                                                      |
| 117                | RESERVED |                                        |                                                                                      |
| 118                | RESERVED |                                        |                                                                                      |
| 119                | RESERVED |                                        |                                                                                      |
| 120                | int      | Last Command Result                    | (0 = Success, >0 = Error)                                                            |
| 121                | int      | Current Tool ID                        |                                                                                      |
| 122                | int      | Raspberry Temperature                  | °C                                                                                   |
| 123                | int      | Raspberry Available Disk Size          | MB                                                                                   |
| 124                | int      | Raspberry Logs Size                    | MB                                                                                   |
| 125                | str      | Hardware Version                       | "hardware version of the robot. Example: "ned2"                                      |
| 126                | str      |                                        |                                                                                      |
| 127                | RESERVED |                                        |                                                                                      |
| 128                | RESERVED |                                        |                                                                                      |
| 129                | int      | System Version Major                   | The four components of a system version                                              |
| 130                | int      | System Version Minor                   |                                                                                      |
| 131                | int      | System Version Patch                   |                                                                                      |
| 132                | int      | System Version Build                   |                                                                                      |
+--------------------+----------+----------------------------------------+--------------------------------------------------------------------------------------+

Dependencies - Modbus TCP Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- `pymodbus library <https://pymodbus.readthedocs.io/en/latest/index.html>`_
- :doc:`../stack/high_level/niryo_robot_msgs`
- :msgs_index:`std_msgs`
