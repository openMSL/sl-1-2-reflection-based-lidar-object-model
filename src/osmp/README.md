# Modular OSMP Framework - *osmp* Folder

The osmp folder contains the packaging. It handles the exchange of OSI messages with the simulation tool and/or other FMUs.

In this folder only the *modelDescription.in.xml* has to be changed by the user. You can add attributes like the author of the model or the description. The model name will automatically be set, depending on the name defined in "src/model/model_name.conf".
