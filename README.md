# fake_obstacle_perception

https://docs.google.com/presentation/d/1a9HhpNVz3dTrvrPWjlhbl21qyUhf8xuZYC3HBtQ7C4o/edit?usp=sharing

## 機能
- maker (rvizプラグイン)
- publisher (rvizプラグイン)
- auto_publisher (ノード)

### maker

#### 機能
- 模擬障害物を設定し、yamlで保存する
- スケール（縦横高さ）、速度、ライフタイムなどを設定可能
- event_captureを使用しwaypointを追加、移動、削除が可能
- 点群地図（/points_map）を読み込んでz座標の補正に対応

#### Subscribe Topic
/rviz/event_capture/mouse (event_capture/MouseEventCaptureStamped)

#### Publish Topic
- /fake_obstacle_maker/marker (visualization_msgs/MarkerArray)：障害物の通る経路
- /fake_obstacle_maker/pointcloud (sensor_msgs/PointCloud2)：障害物の点群（テスト時使用）

### publisher

#### 機能
- makerで作ったyamlファイルを読み込み、障害物を点群として出力
- 複数の障害物を読み込み可能、かつpublishする障害物のオンオフも可能
- 読み込んだ障害物の組み合わせ、および配信トピック名、点群密度をプリセットとしてyamlに保存

#### Publish Topic
- /fake_obstacle_publisher/marker (visualization_msgs/MarkerArray)：各障害物の通る経路
- `[指定したトピック名]` (sensor_msgs/PointCloud2)：障害物情報を統合した点群

### auto_publisher

#### 機能
- publisherで作成したプリセットを読み込んで、点群として出力
- シナリオテストの自動化用

#### Publish Topic
- /fake_obstacle_auto_publisher/marker (visualization_msgs/MarkerArray)
- `[指定したトピック名]` (sensor_msgs/PointCloud2)

#### Parameter
- exec_period：処理周期
- preset_path：読み込むプリセットのパス

#### 使用方法
`$ roslaunch fake_obstacle_perception auto_publisher.launch preset_path:=””`
