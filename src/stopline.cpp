#include "stop_line/stopline_func.hpp"
#include <std_msgs/msg/int16.hpp>
#define LEFT_CAM 0

int first_run = 1;
int count = 100;

class StopLine : public rclcpp::Node
{
public:
  StopLine()
  : Node("find_stop_line")
  {

    distance_pub_ = this->create_publisher<std_msgs::msg::Int16>("where_center_point_needs",10);
    
    cap_left_.open(LEFT_CAM);
    cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    if (!cap_left_.isOpened())
    {
    	RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    	return;
    }

    image_processing();

  }
private:
    cv::VideoCapture cap_left_;
    cv::Mat warp_matrix;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr distance_pub_;

    void image_processing()
    {
        while(rclcpp::ok())
        {
            cv::Mat frame;
            cap_left_ >> frame;
            cv::Mat resized_img;
            cv::resize(frame, resized_img, cv::Size(640, 480), 0, 0);

            // 복사본 생성
            cv::Mat resized_img_clone;
            resized_img_clone = resized_img.clone();

            // 추가 변수 생성
            cv::Mat bird_eye_viewed;
            cv::Mat bird_eye_viewed_clone;

            // 버드아이뷰 이미지 변환
            if(first_run == 1)
            {
                bird_eye_viewed = to_bird_eye_view(resized_img_clone);
                first_run = 0;
            }
            else
            {
                // 변환 매트릭스 타입 확인
                if (warp_matrix.type() != CV_32F && warp_matrix.type() != CV_64F)
                {
                    std::cout << "Converting matrix type to CV_32F" << std::endl;
                    warp_matrix.convertTo(warp_matrix, CV_32F);
                }

                cv::warpPerspective(resized_img_clone, bird_eye_viewed, warp_matrix, cv::Size(resized_img_clone.cols, resized_img_clone.rows));
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
            count = show_return_distance(final, count);

            distance_pub_->publish(count);

            imshow("final", final);
            cv::waitKey(1);
        }
        
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StopLine>());
    rclcpp::shutdown();
    return 0;
}
