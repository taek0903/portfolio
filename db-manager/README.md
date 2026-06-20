# 스마트 물류공장 DB/UI 구축 — Isaac Sim 기반 디지털 트윈

> **기간:** 2025년 5월 중순 ~ 5월 26일 (중도 퇴소)  
> **팀 구성:** 6명  
> **담당:** Firebase DB 설계·구축, React 웹 대시보드 개발  
> **환경:** NVIDIA Isaac Sim 4.5, ROS2, Firebase

---

## 프로젝트 개요

NVIDIA Isaac Sim 환경에서 4종의 이기종 로봇(협동로봇, AMR, Spot, 드론)이 협력하는  
스마트 물류공장 디지털 트윈 시스템 중 **데이터 관리 및 모니터링 UI** 파트를 담당.  
Firebase Firestore 기반 DB를 설계하고, 실시간 웹 대시보드 프로토타입을 완성하였다.

---

## 담당 역할

### 1. Firebase Firestore DB 설계 및 구축

- 물류 시스템에 필요한 데이터 구조 설계 (재고, 로봇 상태, 작업 이력)
- 초기 데이터 등록·초기화·모니터링 스크립트 작성

| 스크립트 | 역할 |
|---|---|
| `setup_inventory.py` | 초기 재고 데이터 등록 |
| `seed_example_data.py` | 예시 데이터 삽입 |
| `reset_inventory.py` | DB 전체 초기화 |
| `monitor.py` | 실시간 터미널 모니터링 |
| `test_connection.py` | Firebase 연결 확인 |

### 2. React 웹 대시보드 개발 (프로토타입 완성)

Firebase Realtime 연동 기반 모니터링 대시보드를 React + TypeScript로 구현.

**주요 화면 구성:**

| 컴포넌트 | 기능 |
|---|---|
| `WarehouseMap` | 창고 내 로봇 위치 실시간 시각화 |
| `RobotCard` | 로봇별 상태 및 배터리 모니터링 |
| `InventoryPanel` | 재고 현황 실시간 조회 |
| `StatusBadge` | 로봇 작동 상태 표시 |
| `BatteryBar` | 배터리 잔량 시각화 |

**기술 스택:**

| 분류 | 기술 |
|---|---|
| 프론트엔드 | React 19, TypeScript, Vite, Tailwind CSS |
| 데이터베이스 | Firebase Firestore |
| 실시간 연동 | Firebase SDK, Custom Hooks |

---

## 시스템 아키텍처

```
Isaac Sim 시뮬레이션
        ↓
  ROS2 브릿지 (robot/)
        ↓
Firebase Firestore ← DB 스크립트 (DB/)
        ↓
  React 대시보드 (UI/)
```

---

## 주요 성과

- Firebase DB 설계부터 초기 데이터 구축까지 단독 완성
- 로봇 4종의 상태를 실시간으로 통합 모니터링하는 웹 대시보드 프로토타입 완성
- 중도 퇴소 전까지 DB 및 UI 핵심 기능 구현 완료

---

## PE 직무 연계 포인트

| 현장 경험 | PE 업무 연결 |
|---|---|
| 실시간 데이터 수집·모니터링 시스템 구축 | 공정 데이터 수집 및 SPC 모니터링 |
| DB 구조 설계 (재고·상태·이력 관리) | 공정 이력 관리, 설비 가동 데이터 관리 |
| 다종 로봇 통합 모니터링 대시보드 | 다수 장비 통합 상태 모니터링 |
