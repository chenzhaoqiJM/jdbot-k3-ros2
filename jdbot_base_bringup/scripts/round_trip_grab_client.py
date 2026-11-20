#!/usr/bin/env python3

import sys
import time

import zmq


class RoundTripGrabClient:
    def __init__(
        self,
        zmq_connect: str,
        grab_duration_sec: float = 3.0,
        wait_interval_sec: float = 0.1,
    ) -> None:
        self.grab_duration_sec = grab_duration_sec
        self.wait_interval_sec = wait_interval_sec

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(zmq_connect)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        print(f'ZMQ PAIR 已连接到 {zmq_connect}')

    def send_message(self, message: str) -> None:
        self.socket.send_string(message)
        print(f'已发送 ZMQ 消息: {message}')

    def wait_for_message(self, timeout_sec: float | None = None) -> str | None:
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec

        while True:
            if deadline is not None and time.monotonic() > deadline:
                print('等待消息超时')
                return None

            events = dict(self.poller.poll(timeout=int(self.wait_interval_sec * 1000)))
            if self.socket in events and events[self.socket] == zmq.POLLIN:
                message = self.socket.recv_string().strip()
                print(f'收到 ZMQ 消息: {message}')
                return message

    def handle_message(self, message: str) -> bool:
        if message == 'start grab':
            print(f'开始执行抓取动作，预计耗时 {self.grab_duration_sec:.1f} 秒')
            time.sleep(self.grab_duration_sec)
            self.send_message('finish grab')
            return True

        if message == 'Release the claws':
            print('收到释放夹爪指令，任务结束')
            return False

        print(f'收到未知消息: {message}')
        return True

    def run(self) -> int:
        print('等待导航任务端指令...')
        while True:
            message = self.wait_for_message()
            if message is None:
                return 1
            if not self.handle_message(message):
                return 0

    def shutdown(self) -> None:
        self.poller.unregister(self.socket)
        self.socket.close()
        self.context.term()


def main() -> int:
    client = RoundTripGrabClient(zmq_connect='tcp://127.0.0.1:5777')

    try:
        return client.run()
    except KeyboardInterrupt:
        print('收到中断，客户端退出')
        return 130
    finally:
        client.shutdown()


if __name__ == '__main__':
    sys.exit(main())