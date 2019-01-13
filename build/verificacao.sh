#!/bin/bash

mkdir resultados
> ./resultados/RESULTADOS.txt

#Exemplo: path="./voxelgrided_testes_FRGC/*"
#         path="../nuvens/*
#OBS: É importante ter '/*' no final para que se a lista de todos os arquivos no diretório.

path="./voxelgrided_testes_FRGC/*"

#Para visualizar: enableVisualization="visualizar" ; Para não visualizar: enableVisualization="QUALQUER OUTRA COISA"
enableVisualization="nao-visualizar"

for file in $path; do
	./shapeOperator $file $enableVisualization -50 -18 13
done
