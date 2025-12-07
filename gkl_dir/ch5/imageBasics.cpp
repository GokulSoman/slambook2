#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv){

    // Read the image in argv[1]

    cv::Mat image;
    image = cv::imread(argv[1]);

    // check if image is loaded

    if (image.data == nullptr) {
        cerr << "file " << argv[1] << " does not exist" << endl;
        return 0;
    }

    cout << "- File " << argv[1] << " loaded." << endl;

    cout << "Number of rows: " << image.rows << ", columns: " << image.cols
         << ", channels: " << image.channels() << endl ;

    cv::imshow("This is the image", image);
    cv::waitKey(0);

    if (image.type() != CV_8UC1 && image.type() != CV_8UC3){
        // Then its not a greyscale image or RGB image

        cout << "Image type is incorrect (not greyscale)" << endl;
        return 0;
    }

    cout << "- Greyscale/RGB image has been loaded" << endl;


    // Measure time to loop through all pizel data

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows ; y++){
        // use ptr to get to each row
        unsigned char *row_ptr = image.ptr<unsigned char>(y);

        for (size_t x=0; x < image.cols ; x++){
            unsigned char *data_ptr = &row_ptr[x * image.channels()];

            for (int c=0; c!= image.channels() ; c++){
                unsigned char data = data_ptr[c];
            }
        } 
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast< chrono::duration<double>>(t2 - t1);

    cout << "Time Used " << time_used.count()*1000 << " milli seconds." << endl;

    // Copying images
    // Copying only be referencing

    cv::Mat another_image =  image;

    // Make a rect in another image. This will get reflacted in orig image

    cout << "- Creating changes in a copy of image shows up in original image" << endl;
    another_image(cv::Rect(0,0,100,100)).setTo(0); // set this pixel range to 0

    cv::imshow("Original image", image);

    cv::waitKey(0);

    cv::Mat image_clone = image.clone();

    image_clone(cv::Rect(100,0,120,120)).setTo(255);

    cout << "An image clone does not translate changes to origingal image" << endl;

    cout << "- Creating a white box in clone image" << endl;

    cv::imshow("Image clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;

    
}


