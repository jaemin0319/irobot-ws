# 모델 출처

- 공식 저장소: https://github.com/frankarobotics/franka_description (Franka Robotics GmbH)
- 태그 2.9.0, 커밋 `7aeeddc449edf8d62b594f9e36a81da53e7796f9` (2026-10-07 `git ls-remote`로 재확인)
- 라이선스: Apache-2.0. 원본 `third_party/franka_description/LICENSE`, `NOTICE` 보존
- 확보 방법: `git clone --filter=blob:none --no-checkout --depth 1 --branch 2.9.0` + cone sparse-checkout
  (`robots/common robots/fr3 end_effectors/common end_effectors/franka_hand meshes/robots/fr3 meshes/robot_ee/franka_hand_white` + 루트 파일)
- 의존성 추적: xacro 전개(`scripts/20_expand_xacro.sh`)가 오류 없이 끝났고, 전개 URDF의 모든 `package://` 메시가 실제 파일로 해석됨(변환 스크립트에서 파일을 열어 사용)
- 비공식 MJCF(mujoco_menagerie)는 사용하지 않음
- 저장소에는 원본 체크아웃을 넣지 않고 `scripts/10_fetch_model.sh`로 같은 커밋을 받습니다. xacro 전개 결과 `models/urdf/fr3_hand.{urdf,srdf}`만 넣었으며, Apache-2.0 고지는 `models/urdf/NOTICE.md`, `LICENSE.franka_description`에 있습니다.

## 체크아웃 파일 sha256 (앞 16자리)
```
4b25434e6ec2012e  LICENSE
13991525876741e3  NOTICE
6ce352414f20a6b2  end_effectors/common/ee_with_one_link.xacro
6edc70dc552caa25  end_effectors/common/franka_hand.xacro
29043c9cbd7aa7e0  end_effectors/common/utils.xacro
aaf62f22ddb75037  end_effectors/franka_hand/franka_hand.srdf.xacro
6369fbff364bb38f  end_effectors/franka_hand/franka_hand.urdf.xacro
496a3b356b31339e  end_effectors/franka_hand/franka_hand_arguments.xacro
6f09ae9ac8838e61  end_effectors/franka_hand/inertials.yaml
94493e94f30fe940  meshes/robot_ee/franka_hand_white/collision/hand.stl
12c7bc9b55ffd0d1  meshes/robot_ee/franka_hand_white/visual/finger.dae
1b50601d3157f810  meshes/robot_ee/franka_hand_white/visual/hand.dae
56990396d712b2eb  meshes/robots/fr3/collision/link0.stl
e023cf1db282e07f  meshes/robots/fr3/collision/link1.stl
3117fd3aea238c07  meshes/robots/fr3/collision/link2.stl
1f21215bdb628922  meshes/robots/fr3/collision/link3.stl
b9574ef02cdf234d  meshes/robots/fr3/collision/link4.stl
c174872b06547dc6  meshes/robots/fr3/collision/link5.stl
9ad5c4698aed204c  meshes/robots/fr3/collision/link6.stl
c48646a72ed0264e  meshes/robots/fr3/collision/link7.stl
3c52a112d1610d64  meshes/robots/fr3/visual/link0.dae
5c5846eabc714aef  meshes/robots/fr3/visual/link1.dae
90a1bd38abdc53d0  meshes/robots/fr3/visual/link2.dae
1c7a2a8124e39ad0  meshes/robots/fr3/visual/link3.dae
dd134cf13cdaced2  meshes/robots/fr3/visual/link4.dae
cbcabd369e044eba  meshes/robots/fr3/visual/link5.dae
907b16fea84931f4  meshes/robots/fr3/visual/link6.dae
b9d05ac502ff269b  meshes/robots/fr3/visual/link7.dae
ff3d3c63eaccfb28  package.xml
47a4625f8df6431f  robots/common/franka_arm.srdf.xacro
3c19225fa490120b  robots/common/franka_arm.xacro
5f9adfd8570b2f6f  robots/common/franka_robot.xacro
5a8c3957a2a9622d  robots/common/group_definition.xacro
b6819d3530d760ae  robots/common/utils.xacro
40fb78a7082e4fb7  robots/fr3/accelerometers.yaml
e65e145114b431bd  robots/fr3/dynamics.yaml
4351fe2a301be007  robots/fr3/fr3.srdf.xacro
8f068b2f9093e36f  robots/fr3/fr3.urdf.xacro
02a39046b36303f7  robots/fr3/inertials.yaml
42e999184f376895  robots/fr3/joint_limits.yaml
44326830a6773dd6  robots/fr3/kinematics.yaml
```
