# Set the components that your project depends on
set(priv_requires "driver")

# Register the component
idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES ${priv_requires})