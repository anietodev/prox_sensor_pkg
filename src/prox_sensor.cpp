// Librerías estándar y de ROS 2
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

// Dependencias de ROS 2 necesarias
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Clase principal del nodo
class PublisherTrajectory : public rclcpp::Node
{
public:
    // Constructor del nodo
    PublisherTrajectory()
    : Node("prox_sensor_node")  // Nombre del nodo en ROS 2
    {
        // Publicador de velocidad final (comando seguro) al tópico "cmd_vel"
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Suscripción a odometría
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PublisherTrajectory::odom_data_callback, this, std::placeholders::_1));

        // Suscripciones a los sensores de rango IR
        ir1_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "ir1", 10, std::bind(&PublisherTrajectory::ir1_data_callback, this, std::placeholders::_1));
        ir2_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "ir2", 10, std::bind(&PublisherTrajectory::ir2_data_callback, this, std::placeholders::_1));
        ir3_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "ir3", 10, std::bind(&PublisherTrajectory::ir3_data_callback, this, std::placeholders::_1));
        ir4_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "ir4", 10, std::bind(&PublisherTrajectory::ir4_data_callback, this, std::placeholders::_1));
        ir5_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "ir5", 10, std::bind(&PublisherTrajectory::ir5_data_callback, this, std::placeholders::_1));
        ir6_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "ir6", 10, std::bind(&PublisherTrajectory::ir6_data_callback, this, std::placeholders::_1));

        // Suscripción a los comandos de teleoperación (sin filtrar)
        cmd_vel_teleop_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_teleop", 10, std::bind(&PublisherTrajectory::cmd_vel_teleop_data_callback, this, std::placeholders::_1));

        // Temporizador para ejecutar periódicamente la función que publica el comando filtrado
        timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PublisherTrajectory::cmd_vel_callback, this));

        // Inicialización de los valores de sensores IR con infinito
        latest_ir1_range = std::numeric_limits<float>::infinity();
        latest_ir2_range = std::numeric_limits<float>::infinity();
        latest_ir3_range = std::numeric_limits<float>::infinity();
        latest_ir4_range = std::numeric_limits<float>::infinity();
        latest_ir5_range = std::numeric_limits<float>::infinity();
        latest_ir6_range = std::numeric_limits<float>::infinity();
    }

private:
    // Variables para guardar el último comando de teleop y lecturas IR
    geometry_msgs::msg::Twist last_cmd_vel_teleop;
    float latest_ir1_range, latest_ir2_range, latest_ir3_range, latest_ir4_range, latest_ir5_range, latest_ir6_range;

    // Función para escalar o anular el comando según proximidad de obstáculos
    geometry_msgs::msg::Twist scale_velocity_by_distance(
        const geometry_msgs::msg::Twist &input_cmd,
        float distance_ir1, float distance_ir2, float distance_ir3,
        float distance_ir4, float distance_ir5, float distance_ir6)
    {
        auto scaled_cmd = input_cmd;

        // Parámetros para seguridad
        const float min_distance = 0.15;  // distancia mínima para detenerse
        const float max_distance = 0.5;   // distancia para empezar a frenar
        const float vel_obst = -0.1;      // velocidad reducida cerca de obstáculos

        if (scaled_cmd.linear.x < 0.0) // Movimiento hacia adelante
        {
            if ((distance_ir1 <= min_distance) || (distance_ir2 <= min_distance) || (distance_ir3 <= min_distance))
            {
                RCLCPP_WARN(this->get_logger(), "¡Obstáculo muy cerca! Parando el robot.");
                scaled_cmd.linear.x = 0.0;
            }
            else if ((distance_ir1 < max_distance) || (distance_ir2 < max_distance) || (distance_ir3 < max_distance))
            {
                RCLCPP_INFO(this->get_logger(), "Reduciendo velocidad.");
                if (scaled_cmd.linear.x < vel_obst)
                    scaled_cmd.linear.x = vel_obst;
            }
        }
        else if (scaled_cmd.linear.x > 0.0) // Movimiento hacia atrás
        {
            if ((distance_ir4 <= min_distance) || (distance_ir5 <= min_distance) || (distance_ir6 <= min_distance))
            {
                RCLCPP_WARN(this->get_logger(), "¡Obstáculo muy cerca! Parando el robot.");
                scaled_cmd.linear.x = 0.0;
            }
            else if ((distance_ir4 < max_distance) || (distance_ir5 < max_distance) || (distance_ir6 < max_distance))
            {
                RCLCPP_INFO(this->get_logger(), "Reduciendo velocidad.");
                if (scaled_cmd.linear.x > -vel_obst)
                    scaled_cmd.linear.x = -vel_obst;
            }
        }

        return scaled_cmd;
    }

    // Callback periódico que publica el comando de velocidad seguro
    void cmd_vel_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg = scale_velocity_by_distance(
            last_cmd_vel_teleop,
            latest_ir1_range, latest_ir2_range, latest_ir3_range,
            latest_ir4_range, latest_ir5_range, latest_ir6_range);
        cmd_vel_publisher->publish(msg);
    }

    // Callback para datos de odometría
    void odom_data_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Odom - Posición (x: %.2f, y: %.2f), Velocidad (vx: %.2f, vy: %.2f), omega_z: %.2f",
                    odom_msg->pose.pose.position.x,
                    odom_msg->pose.pose.position.y,
                    odom_msg->twist.twist.linear.x,
                    odom_msg->twist.twist.linear.y,
                    odom_msg->twist.twist.angular.z);
    }

    // Callbacks para sensores IR
    void ir1_data_callback(const sensor_msgs::msg::Range::SharedPtr ir1_msg)
    {
        latest_ir1_range = ir1_msg->range;
        RCLCPP_INFO(this->get_logger(), "Sensor IR1 - Rango: %.3f m", latest_ir1_range);
    }

    void ir2_data_callback(const sensor_msgs::msg::Range::SharedPtr ir2_msg)
    {
        latest_ir2_range = ir2_msg->range;
        RCLCPP_INFO(this->get_logger(), "Sensor IR2 - Rango: %.3f m", latest_ir2_range);
    }

    void ir3_data_callback(const sensor_msgs::msg::Range::SharedPtr ir3_msg)
    {
        latest_ir3_range = ir3_msg->range;
        RCLCPP_INFO(this->get_logger(), "Sensor IR3 - Rango: %.3f m", latest_ir3_range);
    }

    void ir4_data_callback(const sensor_msgs::msg::Range::SharedPtr ir4_msg)
    {
        latest_ir4_range = ir4_msg->range;
        RCLCPP_INFO(this->get_logger(), "Sensor IR4 - Rango: %.3f m", latest_ir4_range);
    }

    void ir5_data_callback(const sensor_msgs::msg::Range::SharedPtr ir5_msg)
    {
        latest_ir5_range = ir5_msg->range;
        RCLCPP_INFO(this->get_logger(), "Sensor IR5 - Rango: %.3f m", latest_ir5_range);
    }

    void ir6_data_callback(const sensor_msgs::msg::Range::SharedPtr ir6_msg)
    {
        latest_ir6_range = ir6_msg->range;
        RCLCPP_INFO(this->get_logger(), "Sensor IR6 - Rango: %.3f m", latest_ir6_range);
    }

    // Callback para comando de velocidad desde teleoperación
    void cmd_vel_teleop_data_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_teleop_msg)
    {
        last_cmd_vel_teleop = *cmd_vel_teleop_msg;
        RCLCPP_INFO(this->get_logger(), "Teleop - Linear: (x: %.2f, y: %.2f), Angular z: %.2f",
                    cmd_vel_teleop_msg->linear.x,
                    cmd_vel_teleop_msg->linear.y,
                    cmd_vel_teleop_msg->angular.z);
    }

    // Punteros a publicador, suscriptores y temporizador
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir1_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir2_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir3_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir4_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir5_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir6_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_teleop_subscriber;
};

// Función principal del nodo ROS 2
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // Inicializa el sistema ROS 2
    rclcpp::spin(std::make_shared<PublisherTrajectory>());  // Ejecuta el nodo
    rclcpp::shutdown();  // Finaliza el sistema ROS 2
    return 0;
}

