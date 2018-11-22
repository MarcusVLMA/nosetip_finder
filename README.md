# nosetip_finder
Finding nose tips in point cloud images.

O projeto contém um script bash (verificacao.sh) para realizar a execução.
# OBS: As nuvens utilizadas no teste passaram antes pelo pcl_voxel_grid com -leaf 2.5
# Como Fazer
Editar a variável "path" no arquivo "verificacao.sh" onde estão as nuvens a ser analisadas. O resultado aparecerá em um ".txt" onde cada linha contém "nome_da_nuvem.pcd (coordenadas da ponta do nariz)". Além disso, haverá uma pasta no mesmo diretório com vários arquivos ".pcd" de nome "nosetip_nome_da_nuvem.pcd".
# Visualização
Se quiser ir visualizando cada nuvem e sua ponta do nariz basta editar o valor da variável enableVisualization de "nao-visualizar" para "visualizar".
