#!/bin/bash

# Create directories
mkdir -p external/glad/include
mkdir -p external/glad/src

# Clone GLAD repository
git clone https://github.com/Dav1dde/glad.git temp_glad

# Install glad using pip
python3 -m pip install --user glad

# Generate GLAD files
python3 -m glad --profile core --api gl=3.3 --generator c --out-path glad_gen

# Move files to correct locations
cp -r glad_gen/include/* external/glad/include/
cp glad_gen/src/glad.c external/glad/src/

# Clean up
rm -rf temp_glad glad_gen

echo "GLAD setup completed successfully!" 