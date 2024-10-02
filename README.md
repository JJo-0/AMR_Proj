## 프로젝트 개요

### 목차
- [프로젝트 개요](#프로젝트-개요)
  - [목차](#목차)
- [프로젝트 정보](#프로젝트-정보)
- [아이디어 기획](#아이디어-기획)
- [하드웨어](#하드웨어)

---
## 사진   
<img src="https://github.com/JJo-0/JJo-0.github.io/blob/main/image/AMR_.png?raw=true" alt="drawing" style="width:200px;"/>

## 프로젝트 정보

|프로젝트| 설명 | 기타 |
|:---|:---:| ---:|
| 프로젝트 기간| 2023.05.01 ~ | (#아이디어-기획) |
| 프로젝트 성격 | 캡스톤디자인(졸업작품) | |
| 개발 인원 | 팀 / 4명 | [박지호](https://github.com/JJo-0), [이건원](https://github.com/leegunwon), <br> [조성우](https://github.com/), [장영도](https://github.com/) |
| 하드웨어 | raspberry pi 5, mdrobot 등.. [링크](##하드웨어) | |


## 설치 방법 안내

AMR 자율주행을 하고 싶으면 따라하시면 됩니다.
```bash
git clone https://github.com/JJo-0/AMR_Proj.git

cd AMR_Proj
```
   
쓰는 cpu 버전에 따라서 달라진다.  
사용 및 검증 버전   
**arm64(aarch64)**
- raspberry pi 4
- raspberry pi 5
- AGX Xavier
   
**x86_64**
- MINIX (n100 미니 pc)
- Intel Cpu 데스크탑
   
[cpu_아키텍처_종류_설명링크](https://velog.io/@480/%EC%9D%B4%EC%A0%9C%EB%8A%94-%EA%B0%9C%EB%B0%9C%EC%9E%90%EB%8F%84-CPU-%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98%EB%A5%BC-%EA%B5%AC%EB%B6%84%ED%95%B4%EC%95%BC-%ED%95%A9%EB%8B%88%EB%8B%A4)   

|아키텍쳐| ros image | 기타 |
|:---|:---:| ---:|
| arm64 | [ros_foxy image](https://hub.docker.com/r/arm64v8/ros/) |ros 공식 지원 image, (arm64v8)|
| x86_64 | [ros_foxy image](https://hub.docker.com/_/ros) |ros 공식 지원 image, (ros)|
   
   
```bash



## 하드웨어


