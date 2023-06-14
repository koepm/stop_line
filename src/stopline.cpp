#include "stop_line/stopline_func.hpp"

StopLine::StopLine()
: Node("find_stop_line")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&StopLine::image_callback, this, std::placeholders::_1));
}

void StopLine::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // sub 이미지 리사이즈
    cv::Mat resized_img;
    cv::resize(cv_ptr->image, resized_img, cv::Size(640, 480), 0, 0);

    // 복사본 생성
    cv::Mat resized_img_clone;
    resized_img_clone = resized_img.clone();

    // 추가 변수 생성
    cv::Mat bird_eye_viewed;
    cv::Mat bird_eye_viewed_clone;

    // 버드아이뷰 이미지 변환
    if(StopLine::first_run == 1)
    {
        bird_eye_viewed = StopLine::to_bird_eye_view(resized_img_clone);
        StopLine::first_run = 0;
    }
    else
    {
        // 변환 매트릭스 타입 확인
        if (StopLine::warp_matrix.type() != CV_32F && StopLine::warp_matrix.type() != CV_64F)
        {
            std::cout << "Converting matrix type to CV_32F" << std::endl;
            StopLine::warp_matrix.convertTo(StopLine::warp_matrix, CV_32F);
        }

        cv::warpPerspective(resized_img_clone, bird_eye_viewed, StopLine::warp_matrix, cv::Size(resized_img_clone.cols, resized_img_clone.rows));
    }


    // 버드아이뷰 이미지 복사
    bird_eye_viewed_clone = bird_eye_viewed.clone();

    // 원본이미지 사이즈 재변경
    cv::Mat re_resized_img;
	cv::resize(resized_img_clone, re_resized_img, cv::Size(641, 481), 0, 0);

    cv::Mat find_line_result = find_line(bird_eye_viewed_clone, bird_eye_viewed);
    cv::resize(find_line_result, find_line_result, re_resized_img.size());

    cv::Mat final;
    addWeighted(find_line_result, 1, re_resized_img, 1, 0, final);
    StopLine::count = StopLine::show_return_distance(final, count);
	imshow("final", final);
    cv::waitKey(1);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StopLine>());
    rclcpp::shutdown();
    return 0;
}
