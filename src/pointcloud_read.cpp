#include <pointcloud_read.h>

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}


int main(int argc, char *argv[] ){ 
    std::size_t line_num = 0;
    std::stringstream folder_path;
    folder_path << argv[1] << std::setfill('0') << std::setw(6) << line_num << ".bin";
    std::vector<float> lidar_data = read_lidar_data(folder_path.str());

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> laser_cloud;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4)
    {
        pcl::PointXYZI point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        point.intensity = lidar_data[i + 3];
        laser_cloud.push_back(point);
    }

    *ptr_cloud = laser_cloud;

    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(ptr_cloud);

    while (!viewer.wasStopped ())
    {
    }

    return 0;
}
