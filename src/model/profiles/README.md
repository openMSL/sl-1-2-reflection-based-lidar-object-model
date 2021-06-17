# Modular OSMP Framework - *profiles* Folder

The *profiles* folder contains the parts of the parameter profiles, that are strategy-independent.

The overall parameters for the model are declared in *profile.hpp.in*.
It only declares, what parameters there are, but not the actual values.
The values are defined in individual profiles:
For every profile a new profile_*profile_name*.hpp.in is set.
In case of a sensor model, you can for example set individual profiles for different sensor types.

The *profile_list.conf* defines which profiles exist.
Every line in that file denotes one profile.
All profiles have to be included in the *profile_list.hpp*.
