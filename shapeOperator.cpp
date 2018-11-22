#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>


void scaleShapeIndexes(std::vector<float> &shapeIndexes)
{
    float menor;
    float maior;

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        if (i == 0)
        {
            menor = shapeIndexes[i];
            maior = shapeIndexes[i];
        }
        else
        {
            if (shapeIndexes[i] < menor)
            {
                menor = shapeIndexes[i];
            }

            if (shapeIndexes[i] > maior)
            {
                maior = shapeIndexes[i];
            }
        }

    }

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        shapeIndexes[i] = ((((shapeIndexes[i] - menor)/(maior - menor))*(2))-1);
    }
//    if ((-menor) > maior)
//    {
//        for (int i = 0; i < shapeIndexes.size(); i++)
//        {
//            shapeIndexes[i] = shapeIndexes[i]/(-menor);
//        }
//    }
//    else
//    {
//        for (int i = 0; i < shapeIndexes.size(); i++)
//        {
//            shapeIndexes[i] = shapeIndexes[i]/(maior);
//        }
//    }

    std::cout << "Maior: " << maior << std::endl << "Menor: " << menor << std::endl;
}

void connectedComponnent(std::vector<float> &shapeIndexes, int &inicioComponente, int &fimComponente)
{
    int counter = 0;
    int max_counter = 0;
    int ini, max_ini = 0;
    int fim, max_fim = 0;

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        if (!(shapeIndexes[i] > -1.0f && shapeIndexes[i] < -0.625))
        {
            counter = 0;

            if ((shapeIndexes[i+1] > -1 && shapeIndexes[i+1] < -0.625))
            {
                ini = i + 1;
                counter++;
            }

            i = i + 1;
        }
        else
        {
            if (i == 0)
            {
                ini = i;
            }

            fim = i;
            counter++;

            if (counter > max_counter)
            {
                max_counter = counter;
                max_ini = ini;
                max_fim = fim;
            }
        }
    }

    inicioComponente = max_ini;
    fimComponente = max_fim;

}

int main (int, char** argv)
{
    std::vector<std::string> problematicos;
    //==================== INICIALIZANDO NUVENS =============================
    std::vector<int> indices;
    std::string filename;
    filename = argv[1];

    //UTILIZADO PARA TESTES, QUANDO SE TEM AS MARCAÇÕES MANUAIS DO NARIZ
    /*
    pcl::PointXYZ narizReal;
    narizReal.x = std::atof(argv[2]);
    narizReal.y = std::atof(argv[3]);
    narizReal.z = std::atof(argv[4]);
    */

    float raio = 13;
    float limite = 0.84;

    std::cout << "Lendo " << filename << std::endl;
    //std::cout << "Nariz Real: " << narizReal << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltrada (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFinal (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile (filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }

    //std::cout << "Carregado " << cloud->points.size () << " pontos." << std::endl;
    //==========================================================

    //==================== FILTRANDO NUVENS =============================
    pcl::PassThrough<pcl::PointXYZ> passaBanda;
    passaBanda.setInputCloud (cloud);
    passaBanda.setFilterFieldName ("z");
    passaBanda.setFilterLimits (-500000.0, 500000.0); //mudar limits dinamicamente
    passaBanda.filter (*cloudFiltrada);

    std::cout << "Tamanho Útil da Nuvem: " << cloudFiltrada->points.size() << std::endl;
    //==========================================================

    //==================== CALCULANDO NORMAL =============================
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud (cloudFiltrada);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloudNormal (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormalFiltrada (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setRadiusSearch (raio);
    //normalEstimation.setKSearch (13);
    normalEstimation.compute (*cloudNormal);

    //Filtrando NaN da nuvem Normal
    pcl::removeNaNNormalsFromPointCloud (*cloudNormal, *cloudNormalFiltrada, indices);
    //==========================================================

    //Pegando da nuvem de pontos apenas os pontos que também constam na nuvem normal
    for (int i = 0; i < indices.size(); i++)
    {
        cloudFinal->points.push_back(cloudFiltrada->points[indices[i]]);
    }
    //==========================================================


    std::cout << "Normais Calculadas: " << cloudNormalFiltrada->points.size() << std::endl;

    //==================== CALCULANDO CURVATURAS PRINCIPAIS =============================
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    principalCurvaturesEstimation.setInputCloud (cloudFiltrada);
    principalCurvaturesEstimation.setInputNormals (cloudNormal);

    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setRadiusSearch (raio);
    //principalCurvaturesEstimation.setKSearch (10);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*principalCurvaturesCloud);

    pcl::PrincipalCurvatures descriptor = principalCurvaturesCloud->points[0];

    std::cout << "Curvaturas Principais Calculadas: " << descriptor << std::endl;
    //==========================================================

// ======================================================================

    // =============== CALCULANDO SHAPE INDEXES ====================
    std::vector<float> shapeIndexes;
    //std::cout << "WIDTH: " << principalCurvaturesCloud->width << std::endl << "HEIGHT: " << principalCurvaturesCloud->height << std::endl;
    for (int i = 0; i < principalCurvaturesCloud->points.size(); i++)
    {
        float k1 = principalCurvaturesCloud->points[i].pc1;
        float k2 = principalCurvaturesCloud->points[i].pc2;
        if (k2 >= k1)
        {
            float atg = atan((k2 + k1)/(k2 - k1));
            float si = (2/M_PI)*(atg);

            shapeIndexes.push_back(si);
        }
        else
        {
            float atg = atan((k1 + k2)/(k1 - k2));
            float si = (2/M_PI)*(atg);

            shapeIndexes.push_back(si);
        }
    }


    // ======================================================
    int inicioComponente = -1;
    int fimComponente = -1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr teste (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        if ((shapeIndexes[i] > limite) && (shapeIndexes[i] < 1))
        {
            float k1 = principalCurvaturesCloud->points[i].pc1;
            float k2 = principalCurvaturesCloud->points[i].pc2;

            teste->push_back(cloudFiltrada->points[i]);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr noseInput (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> ("199narizMenor.pcd", *noseInput) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return -1;
    }


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(noseInput);
    icp.setInputTarget(teste);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*final);

    pcl::PointXYZ centro;

    for(int i = 0; i < final->points.size(); i++)
    {
        centro.x = centro.x + final->points[i].x;
        centro.y = centro.y + final->points[i].y;
        centro.z = centro.z + final->points[i].z;
    }

    centro.x = centro.x/(final->points.size());
    centro.y = centro.y/(final->points.size());
    centro.z = centro.z/(final->points.size());

    cloudFinal->points.push_back(centro);

    int indice = -1;

    for(int i = 0; i < cloudFinal->points.size(); i++)
    {
        if((cloudFinal->points[i].x == centro.x) && (cloudFinal->points[i].y == centro.y) && (cloudFinal->points[i].z == centro.z))
        {
            indice = i;
        }
    }

    if(indice == -1)
    {
        std::cout << "Não deu bom o centro";
        return -1;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> arvoreKd;
    arvoreKd.setInputCloud(teste);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nuvemDoRaio (new pcl::PointCloud<pcl::PointXYZ>);

    if( arvoreKd.radiusSearch (cloudFinal->points[indice], 13, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for(int j = 0; j < pointIdxRadiusSearch.size(); j++)
        {
            nuvemDoRaio->points.push_back(teste->points[pointIdxRadiusSearch[j]]);
        }

        pcl::PointXYZ noseTip;

        for(int i = 0; i < nuvemDoRaio->points.size(); i++)
        {
            if(i == 0)
            {
                noseTip =  nuvemDoRaio->points[i];
            }
            else
            {
                if(noseTip.z < nuvemDoRaio->points[i].z)
                {
                    noseTip = nuvemDoRaio->points[i];
                }
            }
        }

        //UTILIZADO PARA TESTES, QUANDO SE TEM AS MARCAÇÕES MANUAIS DO NARIZ
        /*
        double distance = sqrt( pow((noseTip.x - narizReal.x), 2) + pow((noseTip.y - narizReal.y), 2) + pow((noseTip.z - narizReal.z), 2) );

        if(distance > 15)
        {
            std::cout << "Nariz não encontrado na nuvem " << argv[1] << std::endl;
            std::ofstream ofs;
            ofs.open("PROBLEMAS_teste.txt", std::ios_base::app);
            if(ofs.is_open())
            {
                ofs << argv[1] << std::endl;
                ofs.close();
            }
            else
            {
                PCL_ERROR("NAO ABRIU");
            }

        }
        */
        std::ofstream ofs;
        ofs.open("./resultados/RESULTADOS.txt", std::ios_base::app);
        if(ofs.is_open())
        {
            ofs << argv[1] << " " << noseTip << std::endl;
            ofs.close();
        }
        else
        {
            PCL_ERROR("NAO ABRIU");
        }
        std::cout << "===================================" << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr noseTipCloud(new pcl::PointCloud<pcl::PointXYZ>);
        noseTipCloud->push_back(noseTip);

        std::string file_name = argv[1];
        for(int l = file_name.size()-1; l > -1; l--)
        {
            if(file_name[l] == '/')
            {
                file_name = "./resultados/nosetip_"+file_name.substr(l+1);
                break;
            }
        }

        std::cout << file_name << std::endl;
        pcl::io::savePCDFile(file_name, *noseTipCloud);

        if (std::strcmp(argv[2], "visualizar") == 0)
        {
            pcl::visualization::PCLVisualizer viewer;
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloudFinal, 255, 0, 0);
            viewer.addPointCloud<pcl::PointXYZ>(cloudFinal, single_color, "Nuvem", 0);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Nuvem");

            std::cout << "NoseTip: " << noseTip << std::endl;

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> nuvemDoRaio_color(nuvemDoRaio, 255, 255, 255);
            viewer.addPointCloud<pcl::PointXYZ>(nuvemDoRaio, nuvemDoRaio_color, "nuvemDoRaio", 0);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "nuvemDoRaio");

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> noseTipCloud_color(noseTipCloud, 255, 0, 255);
            viewer.addPointCloud<pcl::PointXYZ>(noseTipCloud, noseTipCloud_color, "noseTipCloud", 0);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "noseTipCloud");

            /*
            pcl::PointCloud<pcl::PointXYZ>::Ptr narizRealCloud(new pcl::PointCloud<pcl::PointXYZ>);
            narizRealCloud->push_back(narizReal);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> narizReal_color(narizRealCloud, 0, 255, 0);
            viewer.addPointCloud<pcl::PointXYZ>(narizRealCloud, narizReal_color, "narizRealCloud", 0);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 11, "narizRealCloud");
            */
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(final, 255, 255, 0);
            viewer.addPointCloud<pcl::PointXYZ> (final, final_color, "final", 0);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final");

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> second_color(teste, 0, 0, 255);
            viewer.addPointCloud<pcl::PointXYZ>(teste, second_color, "teste", 0);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "teste");

            while (!viewer.wasStopped())
            {
                viewer.spinOnce(100);
            }
        }
    }






    return 0;
}
