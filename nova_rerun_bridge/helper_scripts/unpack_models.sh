#!/bin/bash

for f in draco/*.glb; do
    gltf-transform copy "$f" "./$(basename "$f")"  
done

for f in *.glb; do
    gltf-transform unlit "$f" "$f"  
done