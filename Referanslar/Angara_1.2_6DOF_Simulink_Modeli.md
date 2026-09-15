---
tags:
  - bitirme-projesi
  - simulink
  - 6dof
  - roket
  - plant-modeli
tarih: 2026-06-21
durum: devam-ediyor
---

# Angara 1.2 — 6-DOF Roket Simulink Modeli

## Proje Bağlamı

**Tez:** *"Hierarchical Model Predictive Control for Optimal Vertical Landing of Spacecraft"*
**Kurum:** İTÜ Uzay  Mühendisliği (Bitirme Projesi)
**Rol:** Bu model, hiyerarşik MPC kontrolörü için **non-lineer plant referansı** olarak kullanılır.
**Roket:** Angara 1.2 — Rus orta-sınıf fırlatıcı, URM-1 birinci kademe
**Referans makale:** Zanatta & de Sousa (2015), *"A 6-DOF Rocket Model for Control Analysis"*, IJEAST

---

## Genel Mimari

Model, rijit gövde **6 serbestlik dereceli dinamiği** çözer:

- **3 translasyonel DOF** → konum `[x; y; z]` (yer-sabit çerçeve)
- **3 rotasyonel DOF** → açısal hız `ω = [ωx; ωy; ωz]` + quaternion yönelim `q = [q0; q1; q2; q3]`

### Önemli Modelleme Tercihleri

| Tercih | Açıklama |
|--------|----------|
| Gövde ekseni | `o_z` = roketin uzunlaması (standart dışı) |
| Yönelim | Quaternion (Euler açısı değil — singülariteden kaçınmak için) |
| Başlangıç yönelimi | `q_init = [1; 0; 0; 0]` (tam dikey) |
| Atmosfer | COESA 1976 modeli (`aerolibatmos2`) |
| Aerodinamik | Bu sürümde **dahil edilmedi** (sadece itki + ağırlık) |

> **Neden `o_z` uzunlama?** Standart konvansiyonda (`o_x` = uzunlama) dikey fırlatmada Euler açıları gimbal singülaritesine girer. `o_z` seçimi + quaternion entegrasyonu bu sorunu ortadan kaldırır.

---

## Dosya Yapısı

```
rocket_6dof.slx              ← Ana Simulink modeli
rocket_initialization.m      ← Base workspace parametreleri
```

### Simulink İç Yapısı (SLX = ZIP + XML)

| Dosya             | İçerik                           |
| ----------------- | -------------------------------- |
| `system_root.xml` | Top-level bağlantılar            |
| `system_1.xml`    | Thrust Force and Moments (SID=1) |
| `system_143.xml`  | Mass and Inertia (SID=143)       |
| `system_371.xml`  | Thrust Vectoring (SID=371)       |
| `system_438.xml`  | Angular Motion (SID=438)         |
| `system_495.xml`  | Position (SID=495)               |
| `system_543.xml`  | Attitude (SID=543)               |

---

## Başlangıç Parametreleri (`rocket_initialization.m`)

> Simülasyondan önce **mutlaka** çalıştırılmalı. Tüm değişkenler MATLAB base workspace'e yüklenir.

### Yapısal Parametreler

| Değişken | Değer | Birim | Açıklama |
|----------|-------|-------|----------|
| `m_s` | 39 812 | kg | Yapısal (kuru) kütle |
| `z_CG_s` | 24.73 | m | Yapısal kütle merkezi (nozülden) |
| `L_s` | 41.0 | m | Roket uzunluğu |
| `D_I` | 3.0 | m | Roket çapı |

### İtici Gaz

| Değişken | Değer | Birim | Açıklama |
|----------|-------|-------|----------|
| `m_f_ini` | 35 262 | kg | Başlangıç yakıt kütlesi (RP-1) |
| `m_o_ini` | 92 738 | kg | Başlangıç oksitleyici kütlesi (LOX) |
| `m_p_ini` | 128 000 | kg | Toplam itici |
| `mp_dot` | 533.33 | kg/s | Kütle akış hızı |
| `alfa_f` | 2.63 | — | O/F oranı |

**Türetilen yanma süresi:** `128000 / 533.33 ≈ 240 s`

### Tank CG Sınırları

| Değişken | Değer (m) | Açıklama |
|----------|-----------|----------|
| `z_CG_f_ini` | 7.45 | Yakıt tankı dolu → CG |
| `z_CG_f_fin` | 5.10 | Yakıt tankı boş → CG |
| `z_CG_o_ini` | 18.50 | Oksitleyici dolu → CG |
| `z_CG_o_fin` | 10.00 | Oksitleyici boş → CG |

### İtki ve Eyleyici

| Değişken | Değer | Birim | Açıklama |
|----------|-------|-------|----------|
| `T_SL` | 1.922 × 10⁶ | N | Deniz seviyesi itki (2 motor toplamı) |
| `T_VAC` | 2.085 × 10⁶ | N | Vakum itki |
| `D_T` | 2.0 | m | Motorlar arası mesafe |
| `max_noz_ang_rad` | 0.1396 | rad | Maks gimbal açısı (≈ 8°) |
| `max_noz_AngularVelo` | 0.5585 | rad/s | Maks gimbal açısal hızı |

### Başlangıç Durumu

| Değişken | Değer | Açıklama |
|----------|-------|----------|
| `pos_init` | `[0; 0; 0]` | Fırlatma rampası konumu |
| `q_init` | `[1; 0; 0; 0]` | Tam dikey yönelim |

---

## Altsistem Detayları

### Veri Akışı

```
[Motor_CMD, Ksi_CMD, Zeta_CMD, Zeta_diff]
              │
              ▼
    ┌─────────────────────┐
    │   Thrust Vectoring   │  saturation + rate-limit
    └─────────────────────┘
              │  Ksi, ZetaR, ZetaL
              ▼
    ┌──────────────────────────┐
    │  Thrust Force & Moments  │◄── alt, Z_CG, dT
    │  (COESA + gimbal denklemi)│
    └──────────────────────────┘
         │ Thrust_vec    │ Moment_vec
         ▼               ▼
    ┌──────────┐   ┌──────────────┐
    │ Position │   │ Angular      │◄── Js, Jzz
    │  (F=ma)  │   │  Motion      │
    └──────────┘   └──────────────┘
         │ alt          │ w (omega)
         │              ▼
         │        ┌──────────┐
         │        │ Attitude │
         │        │ (quat ∫) │
         │        └──────────┘
         │              │ DCM
         ▼              ▼
      [pos_out]      [dcm_out]
```

---

### 1. Mass and Inertia (SID=143)

**Giriş:** `Motor_CMD` | **Çıkış:** `mp_ins`, `Z_CG`, `Js`, `Jzz`, `m_tot_ins`

**Anlık propellant kütlesi:**
```
ṁ_p = -mp_dot × Switch(Motor_CMD > 0.5)
m_p  = ∫ ṁ_p dt,   IC = m_p_ini,   sınır: [0, m_p_ini]
```

**Yakıt / oksitleyici ayrımı:**
```
M_F_ins  = m_p × 1/(1 + α_f)
M_OX_ins = m_p × α_f/(1 + α_f)
```

**Anlık CG:**
```
Z_CG_F = z_CG_f_fin + (z_CG_f_ini − z_CG_f_fin) × (M_F_ins / m_f_ini)
Z_CG_O = z_CG_o_fin + (z_CG_o_ini − z_CG_o_fin) × (M_OX_ins / m_o_ini)
Z_CG   = (m_s·z_CG_s + M_F·Z_CG_F + M_OX·Z_CG_O) / m_tot
```

**Eylemsizlik momentleri:**
```
Jzz = (1/8) · m_s · D_I²                           ← boylama (sabit)

Js  = (1/12)·m_s·L_s²                              ← yapısal silindir
    + m_s·(z_CG_s − Z_CG)²                         ← yapısal paralel-eksen
    + M_F·(Z_CG_F − Z_CG)²                         ← yakıt paralel-eksen
    + M_OX·(Z_CG_O − Z_CG)²                        ← oksitleyici paralel-eksen
```

![[roket_cg_animasyon.html]]

---

### 2. Thrust Vectoring (SID=371)

**Giriş:** `Ks_CMD`, `Zeta_CMD`, `Zeta_diff` | **Çıkış:** `Ksi`, `ZetaR`, `ZetaL`

#### Gerçek Sinyal Akışı (SLX'ten doğrulandı)

```
Ks_CMD   ──► RateLimiter ──► Saturation(±limit) ──────────────────────────────► Ksi

                              ┌─ Saturation2(±0.75×limit) ─┐
Zeta_CMD ──► RateLimiter1 ───┤                             ├──► Add  ──► Saturation3(±limit) ──► ZetaR
                              └─ Saturation4(±0.75×limit) ─┘
                                                            └──► Add1 ──► Saturation1(±limit) ──► ZetaL

Zeta_diff ──► Goto/From ──► Gain(0.5) ──┬──► Add  (port 2)
                                         └──► Add1 (port 2)
```

#### Çift Kademeli Saturation Mantığı

> **Bu kısım SLX analizinden türetilmiştir — MD referans dosyasında eksik kalmıştı.**

**Kademe 1 — Zeta_CMD'ye %75 limit** (`Saturation2` / `Saturation4`):
```
Upper =  max_noz_ang_rad × 0.75
Lower = -max_noz_ang_rad × 0.75
```
Müşterek pitch komutuna önceden %75 limit uygulanır. Böylece mixing sonrasında `Zeta_diff/2` eklendiğinde toplam fiziksel limiti aşmaz.

**Mixing:**
```
ZetaR_unsaturated = Zeta_CMD_sat + Zeta_diff/2
ZetaL_unsaturated = Zeta_CMD_sat + Zeta_diff/2
```

**Kademe 2 — Fiziksel nozül limiti** (`Saturation3` / `Saturation1`):
```
Upper =  max_noz_ang_rad
Lower = -max_noz_ang_rad
```
Mixing sonrası her nozüle tam limit tekrar uygulanır. Fiziksel nozül kısıtı garanti altına alınır.

**Neden çift kademe?**
Eğer `Zeta_CMD` tam limite (±8°) giderse ve üzerine `Zeta_diff/2` eklenirse toplam ±8°'yi aşar. %75 ön-saturation bu çakışmayı önler; ikinci saturation ise herhangi bir durumda fiziksel sınırın geçilmesini engeller.

#### Rate Limiter
Her üç kanal için: `±max_noz_AngularVelo = ±0.5585 rad/s`

---

### 3. Thrust Force and Moments (SID=1)

**Giriş (9):** `ZetaR`, `ZetaL`, `Ksi`, `T_SL`, `T_VAC`, `alt`, `Z_CG`, `dT`, `Motor_CMD`
**Çıkış (2):** `Thrust_vec` (3×1), `Moment_vec` (3×1)

**İrtifaya bağlı itki:**
```
T_eff   = T_SL + (T_VAC − T_SL) × (1 − ρ(h)/ρ(0))
T_total = T_eff × Motor_CMD
T_eng   = T_total / 2        ← motor başına
```

**Gövde çerçevesinde itki bileşenleri:**
```
T_x = T_eng·(sin(ξ)·cos(ζR) + sin(ξ)·cos(ζL))
T_y = T_eng·(sin(ζR) + sin(ζL))
T_z = T_eng·(cos(ξ)·cos(ζR) + cos(ξ)·cos(ζL))
```

**Moment vektörü (kol = Z_CG):**
```
M_x =  (T_y) × Z_CG
M_y = -(T_x) × Z_CG
M_z =  T_diff × D_T/2        ← yuvarlanma (diferansiyel itki)
```

---

### 4. Angular Motion (SID=438)

**Giriş:** `Js`, `Jzz`, `Moment_vec` | **Çıkış:** `w`, `w_dot`

**Euler dönme denklemi:**
```
J = diag(Js, Js, Jzz)
ω̇ = J⁻¹ · (M − ω × (J·ω))
ω  = ∫ ω̇ dt,   IC = [0; 0; 0]
```

> **Debug pointi:** `Product2` → `J × J⁻¹` = `eye(3)` olmalı. Sinyal adı: `"eye(3) çıkması lazım"`.

---

### 5. Attitude (SID=543)

**Giriş:** `w` | **Çıkış:** `DCM` (3×3)

**Quaternion türev denklemi:**
```
q̇ = 0.5 · Ω(ω) · q

        ⎡  0   −ωx  −ωy  −ωz ⎤
Ω(ω) = ⎢ ωx    0    ωz  −ωy ⎥
        ⎢ ωy  −ωz    0    ωx ⎥
        ⎣ ωz   ωy  −ωx    0  ⎦

q = ∫ q̇ dt,   IC = q_init = [1; 0; 0; 0]
```

Normalizasyon → `Quaternion Normalize` bloğu (sayısal sürüklenme engellenir)
DCM dönüşümü → `Quaternions to DCM` bloğu

---

### 6. Position (SID=495)

**Giriş:** `Thrust_vec`, `DCM`, `m_tot_ins` | **Çıkış:** `Velocity`, `Position`, `alt`

```
Thrust_world = DCMᵀ · Thrust_body
g(h) = g_0 · (R_E / (R_E + h))²
G_vec = m_tot · g(h) · [0; 0; −1]

a = (Thrust_world + G_vec) / m_tot
v = ∫ a dt          IC = 0
p = ∫ v dt          IC = pos_init
alt = p_z
```

---

## MPC Plant Modeli Olarak Kullanım

**State vektörü (16 boyut):**
```
x = [p_x, p_y, p_z,        ← 3 konum
     v_x, v_y, v_z,        ← 3 hız
     q_0, q_1, q_2, q_3,   ← 4 quaternion
     ωx, ωy, ωz,           ← 3 açısal hız
     m_p]                  ← 1 propellant kütlesi
```

**Kontrol girdileri (4 boyut):**
```
u = [Motor_CMD,    ← 0/1 (on-off; sürekli throttle eklenebilir)
     ξ_CMD,        ← yatış komutu (rad)
     ζ_CMD,        ← müşterek pitch komutu (rad)
     ζ_diff_CMD]   ← diferansiyel yuvarlanma (rad)
```

---

## Simülasyon Konfigürasyonu

| Parametre | Değer |
|-----------|-------|
| Başlangıç | 0 s |
| Bitiş | 100 s |
| Solver | FixedStepAuto |
| Adım boyutu | 0.001 s (1 ms) |
| Pacing | Aktif (gerçek zamanlı) |

---

## Bilinen Sorunlar

### t ≈ 224–230s'de Non-Finite Türev Çökmesi

**Belirti:** Angular Motion integratöründe "non-finite derivative" hatası.

**Olası nedenler (öncelik sırasıyla):**

1. **Goto etiket çakışması** — Angular Motion içindeki `Goto→Js` ve `Goto→Jzz` etiketleri, Mass and Inertia'daki orijinal etiketlerle çakışıyor. Simulink belirsiz davranış gösterebilir. **Çözüm:** Angular Motion içindekileri `Js_internal`, `Jzz_internal` olarak yeniden adlandır.

2. **Motor_CMD t=20s'de kesilmiyor** — Manual Switch yanlış pozisyondaysa motor 240s boyunca çalışır, propellant biter, model bozulur.

3. **Inertia matris hatası** — `Js` veya `Jzz` yanlış bağlandıysa `J⁻¹` patlar.

### `pos_out` Yanlış Bağlı

`From10` (GotoTag=`DCM`) → `ToWorkspace(pos_out)` bağlı. Konum değil DCM loglanıyor. **Çözüm:** Position altsistemi `Position` çıkışına bağla.

### ω Sıfır Kalıyor

`Moment_vec` bağlantısı kopuk veya gimbal açıları başlangıçta 0 olduğundan moment üretilmiyor.

### Debug Adımları

1. `eye(3) çıkması lazım` sinyalini izle → `J × J⁻¹ = I` mı?
2. `Js`, `Jzz`, `Z_CG`, `m_tot` değerlerini t≈224s civarında dump et
3. Goto etiket çakışmasını düzelt
4. `Motor_CMD`'i izle — t=20s'de gerçekten 0'a düşüyor mu?
5. `pos_out` bağlantısını düzelt

---

## Hızlı Başlangıç

```matlab
% 1. Parametreleri yükle
rocket_initialization

% 2. Simulink modelini aç, Manual Switch pozisyonlarını kontrol et
% 3. Simülasyonu çalıştır

% Çıkışlar:
% pos_out  → workspace (⚠️ şu an DCM bağlı, düzeltilmeli)
% dcm_out  → workspace

% Karakteristik değerler:
% T/W (kalkış): 1.922e6 / (167812 × 9.81) ≈ 1.17
% Yanma süresi: ~240 s
% Motor kesme (modelde): 20 s
% Maks gimbal: ±8° (0.1396 rad)
```

---

## Kaynaklar

- Zanatta, R. & de Sousa, M. S. (2015). A 6-DOF Rocket Model for Control Analysis. *International Journal of Engineering Applied Sciences and Technology*, 1(1), 1–6.
- `rocket_6dof.slx` — Simulink model dosyası
- `rocket_initialization.m` — Parametre dosyası

---

## İlgili Obsidian Notları

