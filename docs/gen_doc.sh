#!/bin/bash

# gen_doc.sh - Script to generate documentation using Doxygen

# Exit immediately if a command exits with a non-zero status
set -e

# Path to the Doxygen configuration file
DOXYFILE="docs/Doxyfile"

# Check if Doxyfile exists
if [ ! -f "$DOXYFILE" ]; then
    echo "Error: $DOXYFILE not found. Generating a default one..."
    doxygen -g "$DOXYFILE"
    echo "Please edit the generated Doxyfile and run this script again."
    exit 1
fi

# Run Doxygen
echo "Running Doxygen..."
doxygen "$DOXYFILE"

echo "Documentation generated in the output directory specified in Doxyfile."

