//
//  create by tiezheng yu on 2019-2-21
//
//


#include "common_include.h"
#include "data_loader.h"
#include "bf.h"

int main(int argc, char **argv)
{
    string data_file = "/home/icey/Desktop/project/KITTI/2011_09_26_drive_0001_sync";
    PointCloudLoader::Ptr point_cloud_loader (new PointCloudLoader(data_file));
    ImageLoader::Ptr image_loader (new ImageLoader(data_file));
    Calibration::Ptr calibration (new Calibration);

    pcl::visualization::PCLVisualizer viewer("result");//pcl viewer

    for(int i = 0; i < 153; i++)
    {
        cout << "========== fram number :" << i << "=============================" << endl;
        boost::timer timer;
        
        point_cloud_loader->readKittiPclBinData(i);
        image_loader->readKittiImage(i);
        BF::Ptr bf (new BF);
        bf->calibration_ = calibration;
        bf->raw_cloud_ = point_cloud_loader->raw_cloud_;
        bf->raw_image_ = image_loader->image_;
        bf->BFProcess();
        cout<<"total cost time: "<<timer.elapsed() <<endl;

        // cv::imshow("image", bf->raw_image_);
        // cv::waitKey(0);
        // cv::destroyWindow("image");
        viewer.addPointCloud(bf->result_cloud_,to_string(i));
        viewer.setBackgroundColor(0,0,0);
        viewer.addCoordinateSystem();
        viewer.spin();
        viewer.removeCoordinateSystem();
        viewer.removeAllPointClouds(); 
    }

    return 0;
}
