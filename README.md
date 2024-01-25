# HLK-LD2410 library v 0.9.2

  Name: HLK_LD2410
  
  Purpose: Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor

  @author Dejan Gjorgjevikj
  
  @version 0.9.2, 1/2024

  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 
  todo:
   - better documentation
   - examples
   - support for new firmware
   - support for other (simmilar) radar sensors 

  The code in this library was developed from scratch based on the manufacturer datasheet(s) https://drive.google.com/drive/folders/1p4dhbEJA3YubyIjIIC7wwVsSo8x29Fq-
  
  History of changes:
    14.01.2023 - v0.9.0 initial 
    23.01.2023 - v0.9.1 corrected minor bug in reqVersion and reqParameter that did not work correctly if not called from config mode
    25.01.2023 - v0.9.2 added functions for distance resolution reading and setting, Bluetooh, MAC address, fixed bug in reset function, ...

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.

Released under LGPL-2.1 see https://github.com/Gjorgjevikj/HLK_LD2410/blob/master/LICENSE for full license
