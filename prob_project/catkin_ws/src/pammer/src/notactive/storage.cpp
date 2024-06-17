/*
    Eigen::MatrixXd motionmodel(double &theta, double &v, double &omega, double &dt)
    {
        // Create the Jacobian matrix for the motion model
        Eigen::MatrixXd G(3, 3);

        // Avoid division by zero in the case of omega being very small
        if (fabs(omega) > 1e-5)
        {
            G(0, 0) = 1.0;
            G(0, 1) = 0.0;
            G(0, 2) = -v / omega * cos(theta) + v / omega * cos(theta + omega * dt);

            G(1, 0) = 0.0;
            G(1, 1) = 1.0;
            G(1, 2) = -v / omega * sin(theta) + v / omega * sin(theta + omega * dt);

            G(2, 0) = 0.0;
            G(2, 1) = 0.0;
            G(2, 2) = 1.0;
        }
        else
        {
            G(0, 0) = 1.0;
            G(0, 1) = 0.0;
            G(0, 2) = -v * dt * sin(theta);

            G(1, 0) = 0.0;
            G(1, 1) = 1.0;
            G(1, 2) = v * dt * cos(theta);

            G(2, 0) = 0.0;
            G(2, 1) = 0.0;
            G(2, 2) = 1.0;
        }

        return G;
    }

            // Convert LaserScan to PointCloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (scan_msg->ranges[i] < scan_msg->range_max && scan_msg->ranges[i] > scan_msg->range_min)
            {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * cos(angle);
                point.y = scan_msg->ranges[i] * sin(angle);
                point.z = 0.0;
                cloud->points.push_back(point);
            }
        }
****************************************************************************************************
        // Detect circles
        pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg_circle;
        seg_circle.setOptimizeCoefficients(true);
        seg_circle.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg_circle.setMethodType(pcl::SAC_RANSAC);
        seg_circle.setDistanceThreshold(0.0001);
        seg_circle.setInputCloud(cloud);
        seg_circle.segment(*inliers_circle, *coefficients_circle);

        if (inliers_circle->indices.size() == 0)
        {
            ROS_WARN("Could not estimate a circle model for the given dataset.");
            return;
        }

        double xCoord = coefficients_circle->values[0];
        double yCoord = coefficients_circle->values[1];
        double radius = coefficients_circle->values[2];

        xWorld = xCoord * cos(muPred(2)) - yCoord * sin(muPred(2)) + muPred(0);
        yWorld = xCoord * sin(muPred(2)) + yCoord * cos(muPred(2)) + muPred(1);

        ROS_INFO("Circle detected: center (%f, %f), radius %f", xWorld, yWorld, radius);

        double distance = std::sqrt((xWorld - muPred(0)) * (xWorld - muPred(0)) + (yWorld - muPred(1)) * (yWorld - muPred(1)));
        double angle = std::atan2(yWorld - muPred(1), xWorld - muPred(0));

        sm(0) = distance;
        sm(1) = angle;
        ***********************************************************************
        // Convert LaserScan to PointCloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (std::isfinite(scan_msg->ranges[i]))
            {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * cos(angle);
                point.y = scan_msg->ranges[i] * sin(angle);
                point.z = 0;
                cloud->points.push_back(point);
            }
        }

        // Set up RANSAC for circle segmentation
        pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.segment(*inliers_circle, *coefficients_circle);

        if (inliers_circle->indices.size() == 0)
        {
            ROS_WARN("Could not estimate a circle model for the given dataset.");
            return;
        }

        // Extract circle information
        float circle_x = coefficients_circle->values[0];
        float circle_y = coefficients_circle->values[1];
        float circle_radius = coefficients_circle->values[2];

        float distance_to_circle = sqrt(circle_x * circle_x + circle_y * circle_y);
        float angle_to_circle = atan2(circle_y, circle_x);

        ROS_INFO("Detected circle at distance: %f, angle: %f radians", distance_to_circle, angle_to_circle);
    
*/