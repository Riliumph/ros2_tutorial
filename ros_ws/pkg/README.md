# pkg

## 何のディレクトリか？

ROS2のソースコードをパッケージ毎に管理するディレクトリである。

## ディレクトリツリー

```bash
pkg/
├ package1/
├ ...
└ packageN/
```

## パッケージの作り方

```console
$ ros2 pkg create --build-type ament_cmake --license MIT ${your_package_name}
going to create a new package
package name: ${your_package_name}
destination directory: /workspaces/ros_ws/pkg
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['ubuntu <riliumph@outlook.jp>']
licenses: ['MIT']
build type: ament_cmake
dependencies: []
creating folder ./${your_package_name}
creating ./${your_package_name}/package.xml
creating source and include folder
creating folder ./${your_package_name}/src
creating folder ./${your_package_name}/include/${your_package_name}
creating ./${your_package_name}/CMakeLists.txt
```

### maintainer

maintainer項目のメールアドレスは`git`から[取得している](https://github.com/ros2/ros2cli/blob/a18bc6816ece36dad51a8ea5a2d7a216c473d477/ros2pkg/ros2pkg/verb/create.py#L109)。
