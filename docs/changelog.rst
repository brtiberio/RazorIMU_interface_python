=========
Changelog
=========
:version 0.2:
   Moved from optoparse to argparse module.
   changed Queue to make it compatible with python3 queue. Backwards compatibility is maintained.
   Restructured default location. Moved from Lib folder to base path.
   Moved examples to proper folder. This cause backwards compatibility problems. On import, replace
   ``import Lib.razorIMU`` with simply ``import razorIMU``

:version 0.1: initial release
