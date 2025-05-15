// STL
#include <iostream>
#include <memory>
#include <unistd.h>
// ROS2
#include <rclcpp/rclcpp.hpp>
// 3rd
#include <pqxx/pqxx>

const char* node_name = "your_rclcpp";

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger(node_name);
  try {
    auto db_conn = std::make_unique<pqxx::connection>("host=db"
                                                      " port=5432"
                                                      " dbname=postgres"
                                                      " user=postgres"
                                                      " password=postgres");
    pqxx::work transactor(*db_conn);
    std::string query = "SELECT * FROM students";
    pqxx::result result(transactor.exec(query));
    transactor.commit();
    // ヘッダー（列名）を出力
    for (pqxx::row::size_type col = 0; col < result.columns(); ++col) {
      std::cout << result.column_name(col) << "\t";
    }
    std::cout << std::endl;

    // 各行のデータを出力
    for (const auto& row : result) {
      for (const auto& field : row) {
        std::cout << field.c_str() << "\t";
      }
      std::cout << std::endl;
    }
  } catch (const pqxx::sql_error& e) {
    std::cerr << e.what() << " : SQL->" << e.query() << std::endl;
  } catch (const pqxx::usage_error& e) {
    std::cerr << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
