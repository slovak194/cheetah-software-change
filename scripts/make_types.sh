#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"

cd ../lcm-types
# Clean
rm */*.hpp

# Make
lcm-gen -x *.lcm

# Move
mkdir -p cpp
mv *.hpp cpp


echo -e "${GREEN} Done with LCM type generation${NC}"
