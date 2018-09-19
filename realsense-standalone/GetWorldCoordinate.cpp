/*********************************************************************
 * GetWorldCoordinate.cpp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use th700_realsense file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Yu-Hsien Chang, ISCI, NCTU
 */

/////////////////////////////////////////////////////////////////////////////
// Original source from: 
// Authors
// * Rudy Cazabon
// * Rick Blacker
//
// Dependencies
// * LibRealSense
// * OpenCV
//
/////////////////////////////////////////////////////////////////////////////
// This code sample shows how you can use LibRealSense and OpenCV to display
// both an RGB stream as well as Depth stream into two separate OpenCV
// created windows.
//
/////////////////////////////////////////////////////////////////////////////

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace rs;


// Window size and frame rate
int const INPUT_WIDTH 	= 640;//320;
int const INPUT_HEIGHT 	= 480;//240;
int const FRAMERATE 	= 60;

// Named windows
char* const WINDOW_DEPTH = "Depth Image";
char* const WINDOW_DEPTH2 = "Depth Image2";
char* const WINDOW_RGB	 = "RGB Image";
float scale = 0;
const uint16_t * image;

cv::Mat depth16_deproject;
cv::Mat depth8u;

context 	_rs_ctx;
device* 	_rs_camera = NULL;
intrinsics 	_depth_intrin;
intrinsics  _color_intrin;

rs_intrinsics * depth_intrinsics;
rs_intrinsics * color_intrinsics;
bool 		_loop = true;



// Initialize the application state. Upon success will return the static app_state vars address
bool initialize_streaming( )
{
	bool success = false;
	if( _rs_ctx.get_device_count( ) > 0 )
	{
		_rs_camera = _rs_ctx.get_device( 0 );

		_rs_camera->enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
		_rs_camera->enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );

		_rs_camera->start( );

		success = true;
	}
	return success;
}



/////////////////////////////////////////////////////////////////////////////
// If the left mouse button was clicked on either image, stop streaming and close windows.
/////////////////////////////////////////////////////////////////////////////
static void onMouse( int event, int x, int y, int, void* window_name )
{
	if( event == cv::EVENT_LBUTTONDOWN )
	{
		float pixel[2];
		float point[3];

		pixel[0] = x;
		pixel[1] = y;

		float depth_in_meters = scale * image[x + y*INPUT_WIDTH];

		printf("depth_in_meter(m) = %5.4f || depth_in_meter(mm) = %5.4f \n ", depth_in_meters, depth_in_meters*1000);
		//cout << "depth_in_meter = " << depth_in_meters << "depth_in_mm = " << depth_in_meters*1000 << endl;
		
		rs_deproject_pixel_to_point(point, &_depth_intrin, pixel, depth_in_meters);
		cout << "x(pixel) = " << pixel[0] << endl;
		cout << "y(pixel) = " << pixel[1] << endl;

		printf("x(m) = %5.4f || x(mm) = %5.4f \n", point[0], point[0]*1000);
		printf("y(m) = %5.4f || y(mm) = %5.4f \n", point[1], point[1]*1000);
		printf("z(m) = %5.4f || z(mm) = %5.4f \n", point[2], point[2]*1000);

//		cout << "x(m) = " << point[0] << "x(mm) = " << point[0]*1000 << endl;
//		cout << "y(m) = " << point[1] << "y(mm) = " << point[1]*1000 << endl;
//		cout << "z(m) = " << point[2] << "z(mm) = " << point[2]*1000 << endl;
		//_loop = false;
	}
}



/////////////////////////////////////////////////////////////////////////////
// Create the depth and RGB windows, set their mouse callbacks.
// Required if we want to create a window and have the ability to use it in
// different functions
/////////////////////////////////////////////////////////////////////////////
void setup_windows( )
{
	cv::namedWindow( WINDOW_DEPTH, 0 );
	cv::namedWindow( WINDOW_RGB, 0 );

	cv::setMouseCallback( WINDOW_DEPTH, onMouse, WINDOW_DEPTH );
	cv::setMouseCallback( WINDOW_RGB, onMouse, WINDOW_RGB );
}



/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using OpenCV.
/////////////////////////////////////////////////////////////////////////////
bool display_next_frame( )
{
	// Get current frames intrinsic data.
	_depth_intrin 	= _rs_camera->get_stream_intrinsics( rs::stream::depth );
	_color_intrin 	= _rs_camera->get_stream_intrinsics( rs::stream::color );

	//rs_get_stream_intrinsics(_rs_camera, rs::stream::depth, depth_intrinsics, NULL);
	//rs_get_stream_intrinsics(_rs_camera, rs::stream::color, color_intrinsics, NULL);

	// Create depth image
	cv::Mat depth16( _depth_intrin.height,
					 _depth_intrin.width,
					 CV_16U,
					 (uchar *)_rs_camera->get_frame_data( rs::stream::depth ) );

	// Create color image
	cv::Mat rgb( _color_intrin.height,
				 _color_intrin.width,
				 CV_8UC3,
				 (uchar *)_rs_camera->get_frame_data( rs::stream::color ) );

	// < 800
	depth8u = depth16;
	depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );

	image = (const uint16_t *)_rs_camera->get_frame_data(rs::stream::depth);

	imshow( WINDOW_DEPTH, depth8u );
	cvWaitKey( 1 );

//	imshow( WINDOW_DEPTH2, depth16 );
//	cvWaitKey( 1 );

	cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	imshow( WINDOW_RGB, rgb );
	cvWaitKey( 1 );

	return true;
}



/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main( ) try
{
	rs::log_to_console( rs::log_severity::warn );

	if( !initialize_streaming( ) )
	{
		std::cout << "Unable to locate a camera" << std::endl;
		rs::log_to_console( rs::log_severity::fatal );
		return EXIT_FAILURE;
	}

	scale = _rs_camera->get_depth_scale();

	setup_windows( );

	// Loop until someone left clicks on either of the images in either window.
	while( _loop )
	{
		if( _rs_camera->is_streaming( ) )
			_rs_camera->wait_for_frames( );

		display_next_frame( );
	}

	_rs_camera->stop( );
	cv::destroyAllWindows( );

	return EXIT_SUCCESS;
}
catch( const rs::error & e )
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch( const std::exception & e )
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
