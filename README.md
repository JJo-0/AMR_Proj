## 프로젝트 개요

### 목차
- [프로젝트 개요](#프로젝트-개요)
  - [목차](#목차)
- [프로젝트 정보](#프로젝트-정보)
- [시연 영상](#시연-영상)
- [사진](#사진)
- [설치 방법 안내](#설치-방법-안내)
- [하드웨어](#하드웨어)

---

## 프로젝트 정보

| 프로젝트 | 설명 | 기타 |
|:---|:---:| ---:|
| 프로젝트 기간 | 2023.05.01 ~ 2024.07.24 | (#아이디어-기획) |
| 프로젝트 성격 | 캡스톤디자인(졸업작품) | |
| 개발 인원 | 팀 / 4명 | [박지호](https://github.com/JJo-0), [이건원](https://github.com/leegunwon), [조성우](https://github.com/), [장영도](https://github.com/) |
| 하드웨어 | raspberry pi 5, mdrobot 등.. [하드웨어](#하드웨어) | |

---

## 시연 영상

[![YouTube 시연 영상](https://img.youtube.com/vi/TZNeZHwiGjE/0.jpg)](https://youtu.be/TZNeZHwiGjE)

---

## 사진

![AMR 사진](https://github.com/JJo-0/JJo-0.github.io/blob/main/image/AMR_.png?raw=true)

---

## 설치 방법 안내

AMR 자율주행을 하고 싶으면 아래 절차를 따라하세요.

```bash
git clone https://github.com/JJo-0/AMR_Proj.git
cd AMR_Proj
```

사용하는 CPU 아키텍처에 따라 설치 방법이 다릅니다. 아래는 사용 및 검증된 버전입니다.

### arm64(aarch64)

- raspberry pi 4
- raspberry pi 5
- AGX Xavier

### x86_64

- MINIX (n100 미니 pc)
- Intel CPU 데스크탑

[CPU 아키텍처 종류 설명 링크](https://velog.io/@480/%EC%9D%B4%EC%A0%9C%EB%8A%94-%EA%B0%9C%EB%B0%9C%EC%9E%90%EB%8F%84-CPU-%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98%EB%A5%BC-%EA%B5%AC%EB%B6%84%ED%95%B4%EC%95%BC-%ED%95%A9%EB%8B%88%EB%8B%A4)

| 아키텍쳐 | ros image | 기타 |
|:---|:---:| ---:|
| arm64 | [ros_foxy image](https://hub.docker.com/r/arm64v8/ros/) | ros 공식 지원 image, (arm64v8) |
| x86_64 | [ros_foxy image](https://hub.docker.com/_/ros) | ros 공식 지원 image, (ros) |

---

## 하드웨어


