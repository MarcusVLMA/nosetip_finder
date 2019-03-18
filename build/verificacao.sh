#!/bin/bash

path="./build/individuos_com_problemas/*"
enableVisualization="nvisualizar"
saveLocation="./resultados/"
verifyPath=""

mkdir -p resultados

for paste in $path; do
	paste=$paste"/*"
	for file in $paste; do
		./build/nosetip_finder $file $saveLocation $enableVisualization
	done
done

#for paste in $path; do
#	paste=$paste"/*"
#	for file in $paste; do
#		verify=./build/BASE_ARTIFICIAL/LENDMARKS_OCCLUSIONS_NEUTRALS/${file:54}
#		./build/nosetip_finder $file $enableVisualization $limite_menor $limite_maior $raio $raio_pc $verify
#		let num=$num+1
#	done
#done
