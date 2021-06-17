# Modular OSMP Framework - *model* Folder

The *model* folder contains the actual model logic and is subdevided into *include*, *profiles* and *strategies*. It also contains the *model_name.conf*. In this file the name of the model can be set. This will define the name of the overall fmu package as well as the name inside the model description.

The *include* folder does not have to be changed by the user. It contains the definition of the strategy class and defines framework in which the strategies are called.

The *profiles* folder contains the parameter profiles of the model. All strategy-independent parameters are set there.

The *strategies* folder contains individual folders for every strategy. Inside the modular strategies the actual model code is written.
