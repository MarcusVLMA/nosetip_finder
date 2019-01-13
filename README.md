# nosetip_finder
Finding nose tips in point cloud images.

O projeto contém um script bash (verificacao.sh) para realizar a execução.
This project contains a bash script (verificacao.sh) to do the execution.
# OBS: As nuvens utilizadas no teste passaram antes pelo pcl_voxel_grid com -leaf 2.5
# OBS: pcl_voxel_grid with -leaf 2.5 was applied in the point clouds used in tests before executing.
# Como Fazer | How to do
Editar a variável "path" no arquivo "verificacao.sh" onde estão as nuvens a ser analisadas. O resultado aparecerá em um ".txt" onde cada linha contém "nome_da_nuvem.pcd (coordenadas da ponta do nariz)". Além disso, haverá uma pasta no mesmo diretório com vários arquivos ".pcd" de nome "nosetip_nome_da_nuvem.pcd".

Edit "path" variable in the "verificacao.sh" file to the directory containing your point clouds. The result will appear in a ".txt" file, each line contains "point_cloud_name.pcd (nose tip coordinates)". Furthermore, there will be a paste with many ".pcd" files.
# Visualização | Visualization
Se quiser ir visualizando cada nuvem e sua ponta do nariz basta editar o valor da variável enableVisualization de "nao-visualizar" para "visualizar".

If you wish to visualize each point cloud and the respective nose tip, you should edit the variable enableVisualization from "nao-visualizar" to "visualizar".
