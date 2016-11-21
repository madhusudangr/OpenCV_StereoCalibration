//
//  main.cpp
//  VerizonDisparityMap
//
//  Created by Madhusudan Govindraju on 11/9/16.
//  Copyright © 2016 Madhusudan Govindraju. All rights reserved.
//
//     Program to calculate the disparity map from sereo images
//


#include <iostream>
#include "main.h"
//#include "stereo_calib.cpp"

static int print_help()
{
    std::cout <<
    " Given a list of chessboard images, the number of corners (nx, ny)\n"
    " on the chessboards, and a flag: useCalibrated for \n"
    "   calibrated (0) or\n"
    "   uncalibrated \n"
    "     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
    "         matrix separately) stereo. \n"
    " Calibrate the cameras and display the\n"
    " rectified results along with the computed disparity images.   \n" << std::endl;
    std::cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=../data/stereo_calib.xml>\n" << std::endl;
    return 0;
}

static bool readStringList( const std::string& filename, std::vector<std::string>& l )
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((std::string)*it);
    return true;
}


static void
StereoCalib(const std::vector<std::string>& imagelist, cv::Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true)
{
    if( imagelist.size() % 2 != 0 )
    {
        std::cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }
    
    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:
    
    std::vector<std::vector<cv::Point2f> > imagePoints[2];
    std::vector<std::vector<cv::Point3f> > objectPoints;
    cv::Size imageSize;
    
    int i, j, k, nimages = (int)imagelist.size()/2;
    
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    std::vector<std::string> goodImageList;
    
    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const std::string& filename = imagelist[i*2+k];
            cv::Mat img = cv::imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == cv::Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            std::vector<cv::Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                cv::Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, cv::Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
                                              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        cv::Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                std::cout << filename << std::endl;
                cv::Mat cimg, cimg1;
                cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, cv::Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)cv::waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, cv::Size(11,11), cv::Size(-1,-1),
                         cvTermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    std::cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        std::cout << "Error: too little pairs to run the calibration\n";
        return;
    }
    
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(cv::Point3f(k*squareSize, j*squareSize, 0));
    }
    
    std::cout << "Running stereo calibration ...\n";
    
    cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    cv:: Mat R, T, E, F;
    
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 cv::CALIB_FIX_ASPECT_RATIO +
                                 cv::CALIB_ZERO_TANGENT_DIST +
                                 cv::CALIB_USE_INTRINSIC_GUESS +
                                 cv::CALIB_SAME_FOCAL_LENGTH +
                                 cv::CALIB_RATIONAL_MODEL +
                                 cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                                 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );
    std::cout << "done with RMS error=" << rms << std::endl;
    
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        cv::Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
            fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                 imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    std::cout << "average epipolar err = " <<  err/npoints << std::endl;
    
    // save intrinsic parameters
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
        "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        std::cout << "Error: can not save the intrinsic parameters\n";
    
    cv::Mat R1, R2, P1, P2;
    cv::Rect validRoi[2];
    
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, d.Q,
                  cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
    
    d.focalLength = d.Q.at<double>(2,3);
    d.baseline = d.Q.at<double>(3,2);
    
    fs.open("extrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << d.Q;
        fs.release();
    }
    else
        std::cout << "Error: can not save the extrinsic parameters\n";
    std::cout<<"Finished writing the Files\n";
    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    
    // COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;
    
    cv::Mat rmap[2][2];
    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
    // OR ELSE HARTLEY'S METHOD
    else
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        std::vector<cv::Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
        cv::Mat H1, H2;
        stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
        
        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }
    
    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    std::cout<<"after Undistort rectify Map"<<std::endl;
    cv::Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }
    std::vector<cv::Mat> recitfiedImage;
    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            cv::Mat img = cv::imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);
            cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
            //imshow("Rectified Image",rimg);
            //imshow("Rectified Colour Image",cimg);
            //cv::waitKey(0);
            cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
            if( useCalibrated )
            {
                cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
            }
        }
        
        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
//        std::cout <<"Imshow rectified\n";
//        imshow("rectified", canvas);
//        char c = (char)cv::waitKey();
//        if( c == 27 || c == 'q' || c == 'Q' )
//            break;
    }
}




int my_sterioRectifyImages(int argc, char** argv)
{
    d.leftOriginal = cv::imread( "Left1.png");
    d.rightOriginal = cv::imread( "Right1.png");
    cv::Mat img = cv::imread("sample.jpeg");
    
    cv::Size boardSize;
    std::string imagelistfn;
    bool showRectified;
    cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{s|1.0|}{nr||}{help||}{@input|../data/stereo_calib.xml|}");
    if (parser.has("help"))
        return print_help();
    showRectified = !parser.has("nr");
    imagelistfn = parser.get<std::string>("@input");
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    float squareSize = parser.get<float>("s");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    std::vector<std::string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        std::cout << "can not open " << imagelistfn << " or the string list is empty" << std::endl;
        return print_help();
    }
    
    StereoCalib(imagelist, boardSize, squareSize, false, true, showRectified);
    return 0;

}


void my_computeDisparityMap()
{
    
    cv::Mat left_grey, right_grey;
    cv::Mat left_disp, right_disp, filtered_disp; // 16S
    

    
    d.LeftImageSize =  d.leftOriginal.size();
    d.RightImageSize = d.rightOriginal.size();
    
    // get greyscale images
    cv::cvtColor(d.leftOriginal, left_grey, CV_BGR2GRAY);
    cv::cvtColor(d.rightOriginal, right_grey, CV_BGR2GRAY);
    
    // scale down the image
    if (d.Downscale)
    {
        cv::resize(left_grey, left_grey, cv::Size(), 0.5, 0.5);
        cv::resize(right_grey, right_grey, cv::Size(), 0.5, 0.5);
    }
    
    // compute left disparity map using stereo correspondence algorithm (Semi-Global Block Matching or SGBM algorithm)
    cv::Ptr<cv::StereoSGBM> left_sbm = cv::StereoSGBM::create(d.MinDisparity, d.NumDisparities, d.SADWindowSize);
    left_sbm->setUniquenessRatio(d.UniquenessRatio);
    left_sbm->setDisp12MaxDiff(d.Disp12MaxDiff);
    left_sbm->setSpeckleWindowSize(d.SpeckleWindowSize);
    left_sbm->setP1(d.P1);
    left_sbm->setP2(d.P2);
    left_sbm->setMode(d.Mode);
    left_sbm->compute(left_grey, right_grey, left_disp);
    
    // compute right disparity map
    cv::Ptr<cv::StereoMatcher> right_sbm = cv::ximgproc::createRightMatcher(left_sbm);
    //RightRegionOfInterest = _computeRegionOfInterest(RightOriginal.size(), right_sbm);
    right_sbm->compute(right_grey, left_grey, right_disp);
    
    // applying Global Smoothness
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter = cv::ximgproc::createDisparityWLSFilterGeneric(true);
    filter->setDepthDiscontinuityRadius((int)ceil(0.5*d.SADWindowSize));
    filter->setLambda(d.LambdaValue);
    filter->setSigmaColor(d.SigmaColor);
    
    // compute filtered disparity map
    filter->filter(left_disp, left_grey, filtered_disp, right_disp);
    
    // convert filtered disparity map from 16 bit short to 8 bit unsigned char and normalize values
    double minVal, maxVal;
    cv::minMaxLoc(filtered_disp, &minVal, &maxVal);
    filtered_disp.convertTo(d.Disparity, CV_8UC1, 255 / (maxVal - minVal));
    cv::imshow("Disparity",d.Disparity);
    imwrite("SGBM_sample.png", d.Disparity); //save the disparity for further use
}



void my_create_pointCloud()
{

    //Now creating a Camera Matrix and using that to calculate the pointCloud
    double principalPointLeftX = d.leftOriginal.cols * 0.5; //the principal point is the image center
    double principalPointLeftY = d.rightOriginal.rows * 0.5;
    double principalPointRightX = principalPointLeftX;
    
    cv::Mat Qmatrix(cv::Size(4,4), CV_64F);
    Qmatrix.at<double>(0, 0) = 1.0;
    Qmatrix.at<double>(0, 1) = 0.0;
    Qmatrix.at<double>(0, 2) = 0.0;
    Qmatrix.at<double>(0, 3) = -principalPointLeftX;
    Qmatrix.at<double>(1, 0) = 0.0;
    Qmatrix.at<double>(1, 1) = 1.0;
    Qmatrix.at<double>(1, 2) = 0.0;
    Qmatrix.at<double>(1, 3) = -principalPointLeftY;
    Qmatrix.at<double>(2, 0) = 0.0;
    Qmatrix.at<double>(2, 1) = 0.0;
    Qmatrix.at<double>(2, 2) = 0.0;
    Qmatrix.at<double>(2, 3) = d.focalLength;
    Qmatrix.at<double>(3, 0) = 0.0;
    Qmatrix.at<double>(3, 1) = 0.0;
    Qmatrix.at<double>(3, 2) = 1.0 / d.baseline;
    Qmatrix.at<double>(3, 3) = (principalPointLeftX - principalPointRightX) / d.baseline;// (epipolar plane conversion)
    
    
    //whie projecting the image we should keep in mind  min/max row & column bounds for the template and blocks
    cv::Size sz = d.leftOriginal.size();
    int rowMin = (d.NumDisparities-1)+(d.SADWindowSize/2);
    int rowMax = sz.width-(d.SADWindowSize/2);
    int colMin = (d.SADWindowSize/2);
    int colMax = sz.height-(d.SADWindowSize/2);
    cv::Rect troi (rowMin, colMin, rowMax - rowMin, colMax - colMin);
    d.roi = troi;
    
    //so we have to estimate point cloud for the roi, shouldnt go outside, else the point cloud will have splits, we will be searching for matches outside the search space.
    cv::flip(d.Disparity(troi), d.flippedDisp, 0);
    reprojectImageTo3D(d.flippedDisp, d.pointCloud, d.Q, false, CV_32F);
    //cv::reprojectImageTo3D(d.Disparity, d.pointCloud, Qmatrix,false,CV_32F);
    
    //after getting the point cloud we set some scaling variables to help scale the plot being rendered in OpenGL
    d.oPscale = 2.0f;
    //d.xscale = d.oPscale / d.pointCloud.cols;
    d.xscale = d.oPscale;
    //d.yscale = d.oPscale / d.pointCloud.rows;
    d.yscale =  d.oPscale;
    //d.dscale = d.oPscale / ((d.focalLength*d.baseline)/ d.numDisparity);
    d.dscale =d.oPscale;
}

/* Handler for window-repaint event. Called back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {

    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
    
    // Render the point cloud
    glLoadIdentity();                  // Reset the model-view matrix
    glTranslatef(30.0f, -15.0f, -00.0f);  // Move view out of the screen(-30),  down(-1)
    glRotatef(angle,0.0f, 1.0f, 0.0f);  // Rotate about the (,1,0)-axis
    //gluLookAt(1, 2, 3, /* look from camera XYZ */ 30, 0, 0, /* look at the origin */ 0, 1, 0);
    
    glPointSize(1);
    glBegin(GL_POINTS);

    for(int i=0;i<d.Xpoints.size();i++)
        {
            cv::Vec3f c = d.colour[i];
            glColor3f(c[2], c[1], c[0]);
            glVertex3f(d.Xpoints[i], d.Ypoints[i], d.Zpoints[i]);
        }
    glEnd();
    
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
    
    // Update the rotational angle after each refresh [NEW]
    angle -= 0.50f;
}

void render3D_pointCloud(int argc,char** argv)
{
    //Convert the 3d Disparity Points to an array of positions so that I can display it in OpenGL
    cv::Mat newScale_leftImage;
    //cv::imshow("Original Image",d.leftOriginal);
    cv::waitKey(0);
    d.leftOriginal.convertTo(newScale_leftImage, CV_64FC3 ,1.0/255.0);//the colour scale
    for (int y = 0; y < d.pointCloud.rows; ++y)
    {
        for (int x = 0; x < d.pointCloud.cols; ++x)
        {
            // get point position
            cv::Vec3f pos = d.pointCloud.at<cv::Vec3f>(y, x);
            
            // skip this point if depth is unknown (+-infinity)
            if (pos[2] == -INFINITY || pos[2] == INFINITY)
            {
                continue;
            }
            
             //in matlab the array axis is facing down, in openGL the axis is facing up
            cv::Vec3f imgcolor = d.leftOriginal(d.roi).at<cv::Vec3b>(d.leftOriginal.rows - y - 1, x);

            // convert colors from 0-255 to 0-1
            float r = imgcolor[2] * 1.0f / 255.0f;
            float g = imgcolor[1] * 1.0f / 255.0f;
            float b = imgcolor[0] * 1.0f / 255.0f;
            
            imgcolor[2] = r;
            imgcolor[1] = g;
            imgcolor[0] = b;
            
//            float posx = (pos[0] * d.xscale);
//            float posy = (pos[1] * d.yscale);
//            float posz = (pos[2] * 1 );
            float posx = pos[0];
            float posy = pos[1];
            float posz = pos[2];
            d.Xpoints.push_back(posx);
            d.Ypoints.push_back(posy);
            d.Zpoints.push_back(posz);
            d.colour.push_back(imgcolor);
        }
    }
    
    //initialize the OPEN GL
    glutInit(&argc, argv);            // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
    glutInitWindowSize(640, 480);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutCreateWindow(title);          // Create window with the given title
    glutDisplayFunc(display);       // Register callback handler for window re-paint event
    glutReshapeFunc(reshape);       // Register callback handler for window re-size event
    initGL();                       // Our own OpenGL initialization
    glutTimerFunc(0, timer, 0);     // First timer call immediately [NEW]
    glutMainLoop();                 // Enter the infinite event-processing loop
    
}

int main(int argc, char** argv)
{
    //steps
    //1. Sterio Rectify
    my_sterioRectifyImages(argc, argv);
    //2. Compute Disparity Map
    my_computeDisparityMap();
    //3. Calculate the point cloud
    my_create_pointCloud();
    //4. show the points cloud in OpenGL
    render3D_pointCloud(argc, argv);
    //cv::waitKey(0);
    return 1;
}








