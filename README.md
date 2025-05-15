1. 예제 1번의 subscriber 노드를 수정하여 수신한 영상을 컬러영상 ->그레이영상-> 이진영상으로 변환하고 이진영상을
PC로 전송하여 출력하는 패키지 camera1-1을 작성하라.
+ writer.open(dst, 0, (double)30, cv::Size(640, 360), false);
+ 퍼블리셔 노드는 Jetson 보드에서 camera 패키지의 pub 노드를 실행하라.

https://github.com/user-attachments/assets/6388f1dc-97c8-4bf0-a5af-8e9b34fe110d

2. 예제 1번의 섭스크라이버 노드에서 구독한 영상을 동영상 파일(mp4)로 저장하는 패키지 camera1-2를 작성하라.
+ 실행시 저장을 시작하고 ctrl+c를 누르면 저장을 종료하도록 하라.
![스크린샷 2025-05-15 172054](https://github.com/user-attachments/assets/282293d7-8419-483c-920f-ce800f2e8939)
