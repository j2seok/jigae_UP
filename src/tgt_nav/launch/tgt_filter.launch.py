from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tgt_nav',
            executable='tgt_cmd_filter',
            name='tgt_cmd_filter',
            output='screen',
            parameters=[{
                # 프레임/토픽
                'map_frame': 'map',                          # 변동점
                'base_frame': 'base_link',                   # 변동점
                'plan_topic': '/plan',                       # 변동점
                'cmd_in': '/cmd_vel_raw',                    # 변동점: Nav2 출력 remap 후 입력
                'cmd_out': '/cmd_vel',                       # 변동점: 최종 출력

                # TGT 핵심 파라미터
                'lookahead': 0.8,                            # 변동점
                'heading_tol_enter_deg': 8.0,                # 변동점: 회전→직진 전환 임계
                'heading_tol_exit_deg': 4.0,                 # 변동점: 직진→회전 전환 임계
                'min_hold_time': 0.4,                        # 변동점: 모드 최소 유지시간(s)
                'min_go_distance': 0.12,                     # 변동점: GO모드 최소 전진거리(m)
                'heading_lpf_alpha': 0.2,                    # 변동점: 목표 헤딩 저역통과

                # 제한/데드밴드
                'vx_cap': 0.6, 'wz_cap': 1.5,                # 변동점
                'lin_deadband': 0.02, 'ang_deadband': 0.02,  # 변동점

                # 횡오차 보호장치 (선택)
                'use_cross_track_guard': True,               # 변동점
                'cross_track_max_m': 0.30,                   # 변동점
            }]
        )
    ])
