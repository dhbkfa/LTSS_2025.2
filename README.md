# Mo phong 5 robot song song kham pha ban do luoi chua biet
### Frontier-Based Exploration + A* + Reservation Table (C++ / Python)

Du an mo phong 5 robot cung kham pha mot ban do luoi 2D chua biet truoc, co
the chua mot dich that duoc giau ngau nhien. Cac robot chia se chung mot ban
do da kham pha (SharedMap), dung A* de di toi cac "dich ao" (frontier), va
tranh va cham voi nhau qua mot ReservationTable. Toan bo logic mo phong nam
o C++ (chay song song bang `std::thread`); Python chi dam nhiem giao dien -
xay dung ban do bang tay va hien thi qua trinh mo phong.

Xem `docs/report.md` de biet chi tiet thiet ke va cac quyet dinh ky thuat.

---

## 1. Cau truc thu muc

```
multi_robot_exploration/
├── CMakeLists.txt              # build C++ (simulator + tests tuy chon)
├── cpp/
│   ├── include/                 # toan bo logic dang header-only
│   │   ├── core/                 # Types, Constants
│   │   ├── map/                  # GridMap, TruthMap, SharedMap, Inflation
│   │   ├── robot/                # Robot, RobotState
│   │   ├── sensing/              # SensorModel
│   │   ├── planning/             # AStar, Frontier
│   │   ├── coordination/         # TaskAllocator, ReservationTable, Coordinator
│   │   ├── parallel/             # RobotThreads (5 thread sense+plan song song)
│   │   └── io/                   # JsonLogger, UiProtocol
│   ├── app/main.cpp              # entry point cua simulator
│   └── third_party/nlohmann/     # thu vien JSON (single header, MIT license)
├── python/
│   ├── ui/
│   │   ├── main.py                # app chinh (2 tab: Xay map / Chay mo phong)
│   │   ├── map_builder.py         # tab 1: ve ban do bang tay, xuat JSON
│   │   ├── run_viewer.py          # tab 2: chay C++ nen, doc JSON, ve lai
│   │   └── common.py              # mau sac / hang so dung chung
│   └── bridge/
│       └── cpp_process_bridge.py  # goi C++ binary, theo doi file output
├── config/maps/                 # cac map JSON mau (xem muc 5)
├── tests/                       # 4 bo unit test C++ (test_astar, test_frontier,
│                                   test_collision, test_shared_map)
├── build/                       # da co san simulator + 4 file test da build (Linux x86_64)
└── docs/report.md               # bao cao thiet ke chi tiet
```

---

## 2. Build C++ (bat buoc truoc khi chay)

Du an da duoc build san trong thu muc `build/` (Linux x86_64, GCC 13). Neu
chay tren may/he dieu hanh khac, can build lai:

```bash
cd multi_robot_exploration
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

Yeu cau: CMake >= 3.16, trinh bien dich C++17 (GCC/Clang/MSVC), thread
library (`pthread` tren Linux/macOS, tu dong tren Windows).

De build kem unit test:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
make
ctest --output-on-failure
```

Ket qua mong doi: `100% tests passed, 0 tests failed out of 4`.

### Chay thu simulator truc tiep tu dong lenh (khong can Python UI)

```bash
./build/simulator config/maps/map_random_01.json python/run_output 0 0 5000
```

Tham so theo thu tu: `<map_json> <output_dir> [robot_radius=0] [safety_margin=0] [max_timesteps=200000]`.

Sau khi chay, thu muc `output_dir` se chua cac file `step_000000.json`,
`step_000001.json`, ... (moi file la trang thai 1 timestep) va `meta.json`
(tong so buoc + co `finished`).

---

## 3. Chay giao dien Python

Yeu cau: Python 3.8+, module `tkinter` (thuong co san; tren Ubuntu/Debian
neu thieu thi `sudo apt install python3-tk`).

```bash
cd multi_robot_exploration/python/ui
python3 main.py
```

Giao dien co 2 tab:

**Tab 1 - "Xay dung map"**: giu nguyen tinh than cong cu ve map ban dau cua
nguoi dung (chon kich thuoc, cac hinh dang preset chu nhat/L/T/U/dau cong,
ve/xoa vung active bang chuot, sinh maze ngau nhien bang DFS, ve/xoa tuong
noi bo bang click vao canh o). Diem khac biet so voi ban goc:
- Co the dat **toi da 5 vi tri xuat phat** cho robot (thay vi 1 robot `rx,ry`).
- Co the dat **1 dich that** rieng biet.
- Nut **"Xuat map ra JSON"** se luu ban do hien tai thanh file dung dinh
  dang ma C++ doc duoc, roi tu dong chuyen sang tab 2 voi map vua xuat.

**Tab 2 - "Chay mo phong"**: chon file map JSON (hoac tu dong dien san neu
vua xuat tu tab 1), dieu chinh ban kinh robot / khoang an toan (dung de
inflate vat can), roi nhan **"Chay mo phong"** - luc nay C++ `simulator`
duoc khoi chay o tien trinh nen va ghi JSON ra `python/run_output/`. Cac
nut dieu khien:
- **Phat / Dung**: tu dong chay qua cac timestep theo toc do chon o thanh truot.
- **Step ->** / **<- Step**: di chuyen 1 timestep.
- **Thanh truot Timestep**: keo tha de nhay thang toi bat ky timestep nao
  da duoc ghi (xem lai qua khu).
- **Reset**: dung tien trinh C++ dang chay va xoa toan bo trang thai xem.

Vung **mau toi** tren ban do la phan robot **chua kham pha** (UNKNOWN) -
dung yeu cau "robot chi nen thay phan da kham pha" trong bao cao thiet ke.

---

## 4. Kien truc luong du lieu

```
        (nguoi dung ve map bang tay)
                    |
                    v
        python/ui/map_builder.py
          xuat map_xxx.json
                    |
                    v
   ./build/simulator map_xxx.json output_dir ...
          (C++ chay TOAN BO logic:
           TruthMap, SharedMap, 5 robot,
           A*, Frontier, ReservationTable,
           std::thread, Coordinator)
                    |
                    v  (ghi 1 file JSON / timestep)
       python/run_output/step_NNNNNN.json
                    |
                    v
        python/ui/run_viewer.py
        (CHI doc va ve lai, khong
         tu chay thuat toan)
```

C++ giu hai loai ban do tach biet nghiem ngat:
- **TruthMap**: ban do that, CHI duoc `SensorModel` doc, mo phong cam bien.
- **SharedMap**: ban do 5 robot CUNG xay dung qua cam bien, bao ve bang
  `std::mutex`, day la thu duy nhat A*/Frontier/UI duoc thay.

Dich that chi duoc phat hien khi mot robot **dung dung vao o do** (khong
suy luan tu lich su di chuyen), dung quy tac trong bao cao muc 4.2.

---

## 5. Cac map mau co san (`config/maps/`)

| File | Mo ta |
|---|---|
| `map_sample_small.json` | Map 8x8 don gian, co vi du day du field `walls` - tham khao dinh dang JSON. |
| `map_random_01.json` | Map 12x12 vat can ngau nhien, co dich that. |
| `map_no_target.json` | Map 10x10 KHONG co dich that - kiem tra Mode 2 (phu het map). |
| `map_isolated.json` | Co mot vung bi co lap hoan toan (khong the toi duoc) chua dich - kiem tra robot khong bi treo vinh vien. |
| `map_trapped_robot.json` | Mot robot xuat phat bi vay kin hoan toan - kiem tra co che UNREACHABLE. |
| `map_large.json` | Map 40x40, dung de kiem tra hieu nang (xong duoi 1 giay). |
| `map_inflation_test.json` | Dung de thu nghiem tham so ban kinh robot / khoang an toan (inflation). |

## 6. Dinh dang file map JSON

```jsonc
{
  "width": 8, "height": 8,
  "active":   [[bool x H] x W],   // o co ton tai trong hinh dang map khong
  "occupied": [[bool x H] x W],   // o la vat can
  "walls": {                       // (tuy chon) tuong noi bo theo 4 huong
    "N": [[bool x H] x W], "E": [...], "S": [...], "W": [...]
  },
  "start_positions": [[x,y], ...], // toi da 5 vi tri xuat phat
  "target": [x, y]                 // hoac null neu khong co dich that
}
```

---

## 7. Cac diem thiet ke quan trong (tom tat tu qua trinh phat trien)

Trong qua trinh xay dung va **kiem thu thuc te** (chay tren hang chuc map
ngau nhien, kiem tra va cham tung timestep), mot so loi logic quan trong
da duoc phat hien va sua, duoc giu lai duoi dang unit test trong `tests/`:

- **Frontier phai tinh ca o FREE-chua-VISITED**: vi dich that chi lo ra khi
  robot dung len dung o, cac o "nhin thay tu xa" qua cam bien nhung chua
  duoc ghe tham van phai duoc coi la muc tieu can kham pha tiep.
- **Ngoai le o dich**: nhieu robot duoc phep cung dung chung 1 o dich that
  (muc 5.3 bao cao), ReservationTable phai loai tru o nay khoi luat "trung o".
- **Robot dung yen luon duoc uu tien**: trong ReservationTable, mot o dang
  co robot dung (du robot do co dinh di noi khac) khong duoc coi la "trong"
  cho robot khac vao cho toi khi da xac nhan chac chan no se roi di.
- **Doi cho doi dau (swap) phai dung ca hai**: neu 2 robot du dinh doi cho
  cho nhau trong cung 1 timestep, KHONG the chi cho 1 ben di (vi ben kia
  van dang dung yen dung tai o do) - ca hai phai cho.
- **Chong livelock**: neu mot robot bi tu choi di chuyen qua nhieu lan lien
  tiep (vd ket trong hanh lang hep voi robot khac), no se tam thoi doi
  sang muc tieu khac thay vi cho mai mai.
- **Inflation khong duoc de len vi tri robot dang dung**: neu khong, nhieu
  robot dung gan nhau + gan vat can co the vo tinh tu "khoa" chinh minh.

---

## 8. Gioi han da biet

- `ReservationTable` chay O(n^2) moi timestep - du voi n=5 khong thanh van
  de, nhung khong nen tang so robot len qua lon (vai chuc) ma khong toi uu lai.
- `Frontier::findAll` quet toan bo luoi moi timestep (don gian, de hieu)
  thay vi cap nhat incremental - voi map vai tram x vai tram o van du nhanh
  (<1s ca qua trinh), nhung map rat lon (nghin x nghin o) se can toi uu them.
- Giao tiep C++ <-> Python qua file JSON moi timestep, don gian va de debug
  nhung khong toi uu cho map cuc lon chay rat nhieu buoc (hang chuc nghin
  file nho). Phu hop pham vi do an hoc thuat / demo.
