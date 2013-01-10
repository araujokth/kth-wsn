The code for original CTP on CSMA is in folder /tinyos-2.x_CTP_CSMA
  This repository contains the Water Tank application (apps/EasyCollection)

The code for the optimized CTP on TDMA is in folder /tinyos-2.x_optCTP_TDMA
  This repository contains:
    1) the Water Tank application (apps/EasyCollection_WT);
    2) the application to test ring-topology network (apps/EasyCollection), which could also be configured to test two-path-topology network

To use either of above tinyos repositories, rename the existing folder local/src/tinyos-2.x/ to something else, then copy one repository above to local/src/ and rename it to "tinyos-2.x". An alternative way to use the repositories is to modify the default environment variables/PATHs for tinyos in the ~/.bashrc file. The first approach is recommended though.

For more information with these applications, please refer to the README file in the application folder.
