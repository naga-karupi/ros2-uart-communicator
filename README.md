# uart-communicator

## テスト済み環境

| OS           | ros distro |
| ------------ | ---------- |
| ubuntu 20.04 | foxy       |

## 使い方

### ビルド

任意のディレクトリで

```sh
$ mkdir src; cd src
$ git clone git@github.com:naga-karupi/ros2-uart-communicator.git
$ cd ../../
$ colcon build --symlink-install
```

### 実行

```sh
$ source install/setup.bash
$ ros2 run uart_communicator main
```

開始時にポートを開くので、同時にパラメータを設定したほうが楽に設定できる

よって以下のように`launch`ファイルを書いたほうがいいだろう

```py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='uart_communicator',
                executable='main',
                exec_name='uart_communicator',
                name='uart_communicator',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"serial_name": "your port name"}# serial_nameはパラメータ名なので変更しないこと
                ]
            ),
        ]
    )
```

また、後述するが、このように書かなくてもあとからポートを設定する方法は存在する

## 送受信するメッセージ

このノードがどのようなものを送受信しているかまとめた表を示す

| pubsub | topic名         | 型                |
| ------ | --------------- | ----------------- |
| sub    | `uart_msg`      | `UInt8MultiArray` |
| pub    | `uart_rx_msg`   | `UInt8MultiArray` |
| pub    | `uart_fail_msg` | `Bool`            |
| sub    | `uart_reopen`   | `Bool`            |

トピック名に書いてある通りである。

## パラメータ

`serial_name`: デバイス名を入れる

シリアルを開くときに参照するので、それより前に設定しないと設定されない
