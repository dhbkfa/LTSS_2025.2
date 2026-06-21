# Bao cao tham van thiet ke he thong 5 robot kham pha map chua biet bang A*

> **Ghi chu trien khai**: day la ban bao cao thiet ke goc da duoc tham van va
> thong nhat truoc khi xay dung. He thong da duoc trien khai **dung theo
> kich ban duoc mo ta trong tai lieu nay** (muc 5, 6, 7, 9), voi mode dung
> Mode 3 (muc 5.3) nhu khuyen nghi. Cac diem can sua trong tu duy thuat toan
> (muc 4) deu da duoc ap dung: dich ao la frontier thay vi tam ban do co
> dinh (muc 4.1), dich that chi duoc xac nhan khi robot dung dung len o do
> (muc 4.2), va A* chi giai bai toan tim duong toi 1 muc tieu da chon, tach
> biet voi thuat toan exploration o tang cao hon (muc 4.3).
>
> Trong qua trinh xay dung va kiem thu thuc te tren hang chuc map ngau
> nhien, mot so van de phat sinh ngoai du kien ban dau cua bao cao da duoc
> phat hien va xu ly bo sung - xem muc "Bo sung sau trien khai" o cuoi file
> nay va muc 7-8 cua README.md o thu muc goc du an.

## 1. Muc tieu bai toan

De tai huong toi mo phong 5 robot cung kham pha mot ban do chua biet truoc.
Trong ban do co the co mot dich that duoc dat ngau nhien, nhung robot khong
biet vi tri dich. Truong hop khong co dich that, cac robot phai tiep tuc
kham pha cho den khi toan bo vung co the di duoc da duoc phu.

Moi truong ban dau khong duoc cung cap truc tiep cho robot. Simulator co
the biet ban do that de sinh cam bien, nhung thuat toan robot chi duoc
phep biet thong tin thu duoc trong qua trinh di chuyen.

Muc tieu chinh:

* 5 robot chay song song.
* Cung chia se ban do da kham pha.
* Dung A* de tim duong den muc tieu tam thoi.
* Dung co che tranh va cham giua cac robot.
* Co the hien thi qua trinh kham pha bang UI Python.
* Thuat toan loi viet bang C++ de phu hop mon lap trinh song song.

## 2. Pham vi nen chot

Bai toan nen duoc gioi han o dang ban do luoi 2D. Moi truong that co the co
hinh dang bat ky, nhung khi dua vao mo phong phai duoc roi rac hoa thanh
cac o vuong nho.

Moi o co the thuoc mot trong cac trang thai:

* UNKNOWN: robot chua biet.
* FREE: robot biet co the di.
* OCCUPIED: vat can hoac tuong.
* INFLATED_OBSTACLE: vung cam sau khi cong ban kinh robot va khoang an toan.
* VISITED: da co robot di qua.
* TARGET: dich that, chi duoc phat hien khi robot dung vao o do hoac cam
  bien phat hien theo quy tac mo phong.

Quy tac vung cam:

```text
Vung cam = vat can + ban kinh robot + khoang an toan
```

Quy tac nay rat quan trong. Neu khong mo rong vat can, A* co the tim duoc
duong tren ban do luoi nhung robot that hoac robot mo phong co kich thuoc
se va vao bien tuong.

## 3. Danh gia code hien tai

### 3.1. Code Python hien tai

Code Python hien tai phu hop de lam UI thu nghiem mot robot. No co cac
chuc nang tot: tao map kich thuoc tuy chinh, nhieu preset hinh dang, cho
phep ve vung active/inactive, sinh maze bang DFS, chay A* tung buoc, hien
thi robot/duong di/o da di qua/dich.

Tuy nhien, no chua phu hop truc tiep voi yeu cau 5 robot vi: chi co mot
robot voi bien `rx, ry`; chi co mot goal; chua co `RobotState[5]`; chua co
ban do chung cho nhieu robot; chua co reservation table de tranh va cham;
UI dang hien thi toan bo tuong that trong khi robot theo yeu cau chi nen
thay phan da kham pha; A* hien di den goal co dinh, chua phai di den
frontier; chua co co che target that bi an.

Ket luan: file Python nen giu lai lam nen UI, nhung phai tach khoi thuat
toan. Python chi nen ve map, nhan trang thai tu C++, va hien thi qua trinh
chay.

### 3.2. Code C++ flood fill hien tai

Code C++ hien tai co gia tri vi da the hien tu duy micromouse: co ma tran
trong so, co luu tuong theo 4 huong, co cap nhat tuong bang cam bien trai/
phai/truoc, co chon huong di dua tren gia tri nho nhat, co y tuong dich ao,
co doi dich ao sang o chua tham khi chua tim thay dich that.

Tuy nhien, code nay chua nen mo rong truc tiep thanh 5 robot vi: dung bien
global co dinh 16x16; chi co mot robot; khong co lop `Robot`; khong co lop
`SharedMap`; `Direction_XY` chi dung kieu bool nen chua phan biet ro
UNKNOWN/OPEN/WALL; dich ban dau dang hard-code; co che nhan dien dich that
bang queue 5 buoc chua chac dung ve mat logic; chua co xu ly xung dot khi
nhieu robot cung muon vao mot o; chua co phan cong vung kham pha.

Ket luan: nen lay y tuong flood fill/dich ao de giai quyet bai toan robot
khong biet duoc dich o dau trong ban do, nhung khi xay he thong 5 robot
nen viet lai kien truc C++ theo huong module hoa.

## 4. Van de quan trong can sua trong tu duy thuat toan

### 4.1. Khong nen coi "tam ban do" la dich ao co dinh

Neu robot khong biet kich thuoc map, no khong the biet tam ban do. Do do,
dung tam lam dich ao chi hop ly khi simulator cho biet kich thuoc map.

Cach dung hon:

```text
Dich ao = frontier gan nhat hoac frontier co loi nhat
```

Frontier la o da biet nhung nam canh vung chua biet. Robot di den frontier
de mo rong ban do bang nhung o lan can, dieu kien bo sung co the huong ve
huong robot de robot co kha nang tim kiem nhanh hon.

### 4.2. Dich that khong the suy luan tu hinh dang duong di

Neu trong map chi co mot cham dich that, robot chi co the phat hien dich
khi: no dung vao dung o dich, hoac cam bien cua simulator tra ve tin hieu
`isTargetDetected`.

Khong dung lich su 5 buoc de doan dich that. Lich su buoc di co the giup
phat hien vong lap, nhung khong du de ket luan do la dich.

### 4.3. A* khong phai thuat toan kham pha map

A* chi giai bai toan: tu vi tri hien tai den mot muc tieu da chon, duong
nao ngan nhat? Muon kham pha toan bo map, can them thuat toan exploration:

```text
Sense -> Update shared map -> Find frontiers -> Assign frontiers -> A* -> Move -> Repeat
```

## 5. Kich ban thuat toan de xuat

### 5.1. Du lieu ban dau

Simulator giu ban do that, nhung robot khong duoc doc truc tiep. Robot chi
biet: vi tri xuat phat, huong ban dau, kich thuoc mot buoc luoi, thong tin
cam bien tai vi tri hien tai. Robot khong biet: kich thuoc toan map, hinh
dang map, vi tri dich that, co dich that hay khong.

### 5.2. Vong lap chinh

```text
Buoc 1: 5 robot cam bien vung xung quanh.
Buoc 2: Cap nhat SharedMap.
Buoc 3: Kiem tra robot nao dang dung tren dich that.
Buoc 4: Tim danh sach frontier.
Buoc 5: Phan cong frontier cho tung robot.
Buoc 6: Moi robot chay A* den frontier duoc giao.
Buoc 7: CollisionManager kiem tra va cham.
Buoc 8: Robot di chuyen mot o.
Buoc 9: UI cap nhat trang thai.
```

### 5.3. Dieu kien dung

```text
Mode 1: Tim dich
khi mot robot tim thay dich that, thi nhung con robot con lai cap nhat lai
dich that va chay den dich that do va dung, cac robot co the dung cung o dich.

Mode 2: Phu map
Tiep tuc chay cho den khi khong con frontier nao co the tiep can.

Mode 3: Ket hop
Neu co dich that thi dung khi tim thay.
Neu khong co dich that thi dung khi phu het map.
```

Mode 3 la hop ly nhat va da duoc chon lam mode chinh thuc cua he thong.

## 6. Thiet ke song song cho 5 robot

Dung `std::thread` cho 5 robot, moi robot la mot luong lap ke hoach rieng.
Sau do co mot `Coordinator` dong bo theo tung timestep.

```text
Thread robot 1..5: sense + propose path

Coordinator:
- gom de xuat di chuyen
- kiem tra va cham
- cap quyen di chuyen
- cap nhat SharedMap
```

Can co `mutex` de bao ve SharedMap. Khong nen de 5 thread tu y ghi map va
tu di chuyen cung luc, vi se de loi race condition. Khi co kha nang va
cham thi co che dung doi 1 nhip, hoac di duong khac de tranh va cham giua
cac robot.

## 7. Tranh va cham

### 7.1. Trung o

Hai robot khong duoc o cung mot o tai cung mot timestep.

### 7.2. Doi cho doi dau

Hai robot khong duoc doi cho truc tiep trong mot timestep.

Cach xu ly: robot uu tien cao hon duoc di; robot uu tien thap hon cho mot
buoc; hoac robot uu tien thap hon chay lai A* voi constraint.

## 8. Cau truc thu muc de xuat

(Xem cau truc thuc te da trien khai trong README.md o thu muc goc - bam
sat cau truc de xuat trong ban thiet ke nay, voi mot vai thu muc con duoc
gop lai cho gon vi pham vi do an khong can tach qua chi tiet.)

## 9. Cach chia Python va C++

C++ nen giu toan bo logic: map that, map da biet, 5 robot, A*, frontier,
collision, thread, log trang thai.

Python chi nen lam UI: ve ban do, ve robot, ve o da biet/chua biet, ve
duong di, nut Start/Pause/Step/Reset, doc du lieu tu C++ qua JSON.

```text
C++ simulation core  ->  JSON state  ->  Python UI
Python UI            ->  command     ->  C++ simulation core
```

## 10. Ket luan thiet ke

Khong nen mo rong truc tiep code flood fill hien tai thanh 5 robot. Nen
viet lai theo kien truc module hoa, nhung giu lai cac y tuong chinh: ban
do luoi, tuong theo 4 huong, robot chi biet thong tin sau khi cam bien,
dich ao, cap nhat lai duong di sau moi buoc.

Diem can thay doi lon nhat: dich ao khong nen la tam ban do co dinh - dich
ao nen la frontier duoc phan cong cho tung robot.

---

## Bo sung sau trien khai

Cac van de sau khong duoc neu chi tiet trong ban thiet ke goc nhung phat
sinh trong qua trinh trien khai va kiem thu thuc te, da duoc xu ly:

1. **Frontier phai bao gom ca o FREE chua duoc VISITED.** Ban thiet ke goc
   (muc 4.1) chi mo ta frontier la "o da biet ke o chua biet". Tren thuc
   te, vi cam bien co tam nhin (sensor range), mot o co the duoc biet la
   FREE tu xa ma chua co robot nao thuc su dung len no. Vi dich that chi
   lo ra khi robot dung dung vao o (muc 4.2), neu khong coi nhung o FREE
   chua-VISITED nay la frontier thi he thong co the ket luan "da phu het
   map" truoc khi thuc su kiem tra het moi o co kha nang la dich.

2. **Ngoai le "nhieu robot cung o dich" phai duoc ReservationTable xu ly
   tuong minh.** Muc 5.3 yeu cau cac robot co the dung chung o dich, nhung
   neu ReservationTable ap dung dong nhat luat "mot o chi mot robot" se
   gay deadlock ngay tai o dich (robot den sau khong bao gio vao duoc vi o
   "da co nguoi"). Da them ngoai le rieng cho o dich da xac nhan.

3. **Doi cho doi dau phai khien CA HAI robot dung lai**, khong phai chi
   mot ben nhuong. Neu chi mot ben (uu tien thap hon) bi tu choi trong khi
   ben kia (uu tien cao hon) van duoc di, robot uu tien cao se di thang
   vao o ma robot kia (bi tu choi, nen van dung yen) dang chiem - gay va
   cham that. Day la diem tinh vi nhat trong toan bo logic ReservationTable.

4. **Vi tri robot dang dung phai duoc loai tru khoi vung INFLATED_OBSTACLE.**
   Neu inflate vat can theo ban kinh robot ma khong loai tru vi tri robot
   hien tai, cac robot dung gan nhau hoac gan vat can co the vo tinh tu
   bien ban than minh thanh "khong the di duoc", gay deadlock ngay tu dau.

5. **Co che chong livelock cho robot bi ket lien tuc.** Trong hanh lang
   hep voi nhieu robot, co truong hop 2 robot lien tuc tranh chap cung 1
   huong di ma khong ai tien trien duoc qua nhieu timestep. He thong them
   co che: neu mot robot bi tu choi di chuyen qua mot nguong lien tiep, no
   se tam thoi tranh muc tieu hien tai va thu huong khac.

6. **Robot bi co lap hoan toan voi dich (UNREACHABLE)** can duoc danh dau
   rieng de khong chan dieu kien dung toan he thong - neu khong, mot robot
   bi vay kin se khien toan bo mo phong khong bao gio ket thuc du 4 robot
   con lai da xong viec.

Toan bo 6 diem tren deu duoc kiem chung bang unit test rieng trong thu muc
`tests/` va da duoc xac nhan dung qua kiem thu tren hon 60 ban do ngau
nhien khac nhau (xem README.md muc 7-8).
