#!/bin/bash

# This script validates that the project will build before attempting
# to package it


ue5 setroot /home/ue4/UnrealEngine 
ue5 clean

# Do the actual build
ue5 build
