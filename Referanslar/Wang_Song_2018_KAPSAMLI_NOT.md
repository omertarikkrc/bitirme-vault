---
tags:
  - makale-analizi
  - wang-song-2018
  - convex-mpc
  - roket-inisi
  - SOCP
  - SCP
  - hiyerarsik-mpc
  - 6dof
kaynak: "Wang, C. & Song, Z. (2018). Convex Model Predictive Control for Rocket Vertical Landing. *Proceedings of the 37th Chinese Control Conference*, Wuhan, pp. 9837–9842."
ilgili: "[[Angara_1.2_6DOF_Simulink_Modeli]]"
durum: makale-analizi-tamamlandi
---

# Wang & Song (2018) — Convex MPC for Rocket Vertical Landing
## Kapsamlı Makale Analizi ve 6-DOF Entegrasyon Notu

> **Appendixler:** [Kinematik](Appendixes/wang_song_2018_kinematik.html) · [Alfa Geometrisi](Appendixes/wang_song_2018_alpha_geometrisi.html) · [Gamma Kuvvet Dengesi](Appendixes/wang_song_2018_gammadot.html)

---

# 1. Problem/Hedef Tanımı

## 1.1 Motivasyon

Yeniden kullanılabilir roket ekonomisi (SpaceX Falcon 9, 2017 itibarıyla 19 başarılı iniş), birinci kademenin geri kazanımına dayanıyor. Bu, **iki eş zamanlı gereksinim** doğuruyor: güvenli iniş **ve** yakıt optimizasyonu — çünkü geri kazanılan her kilogram yakıt, sonraki fırlatmaya taşınan ek payload demek.

## 1.2 Neden Klasik Yöntemler Yetersiz

**Pontryagin Maksimum Prensibi (PMP)** — klasik optimal kontrol yaklaşımı, makale tarafından reddediliyor:

$$H(x, u, \lambda, t) = \lambda^T f(x, u) + L(x, u)$$

$$\dot{x} = \frac{\partial H}{\partial \lambda}, \quad \dot{\lambda} = -\frac{\partial H}{\partial x}, \quad u^* = \arg\min_u H$$

**Sorun:** Bu, bir **Two-Point Boundary Value Problem (TPBVP)** doğuruyor — $x(t_0)$ başlangıçta, $\lambda(t_f)$ sonda tanımlı. Çözüm için *shooting method* gerekiyor: $\lambda(t_0)$ tahmin et → ileri entegre et → $t_f$'deki koşulu kontrol et → uymuyorsa düzelt → tekrarla. Bu yöntem başlangıç tahminine aşırı hassas, yavaş ve nonlineer sistemlerde yakınsama garantisi yok.

## 1.3 Önerilen Alternatif — Konveks Optimizasyon Hiyerarşisi

| Kavram | Rolü |
|---|---|
| **CGC** (Computational Guidance and Control, Lu) | Her güdüm döngüsünde onboard optimizasyon felsefesi |
| **Lossless Convexification** (Liu) | Dışbükey problemin optimal çözümü = orijinal nonkonveks problemin optimal çözümü (matematiksel garanti) |
| **SCP** (Sequential Convex Programming) | İteratif doğrusallaştır → konveks çöz → yakınsayana kadar tekrarla |
| **SOCP** (Second-Order Cone Programming) | $\|Ax+b\|_2\leq c^Tx+d$ formundaki kısıtlar, polynomial zamanda çözülür |

**En yakın öncül — Liu [11]:** Aynı longitudinal düzlem problemi, SOCP formülasyonu, ama **sabit** terminal zaman ve MPC yok. Wang & Song bu çalışmaya **serbest terminal zaman + MPC + novel receding horizon** ekliyor.

---

# 2. Matematiksel Formülasyon — §2.1 Dinamik Denklemler

## 2.1 Temel Varsayımlar

- İtki yönü daima roket ekseni yönünde (ayrı nozzle deflection modeli yok)
- Silindirik gövde: $C_L=0$, yalnızca sürükleme
- Uzunlamasına düzlem (2-DOF): dikey $r$, yatay $s$

## 2.2 İçsel Koordinat Sistemi (Intrinsic Coordinates)

$$\hat{e}_t = \begin{pmatrix}\sin\gamma\\\cos\gamma\end{pmatrix}_{(r,s)} \quad(\text{teğet, hız yönü}), \qquad \hat{e}_n = \frac{d\hat{e}_t}{d\gamma}=\begin{pmatrix}\cos\gamma\\-\sin\gamma\end{pmatrix}_{(r,s)} \quad(\text{normal, hıza dik})$$

Doğrulama: $\hat{e}_t\cdot\hat{e}_n = \sin\gamma\cos\gamma-\cos\gamma\sin\gamma=0$ ✓

## 2.3 Türetim — $\dot{r},\dot{s}$ (Kinematik)

$$\vec{V}=V\hat{e}_t \;\Rightarrow\; \boxed{\dot{r}=V\sin\gamma,\quad \dot{s}=V\cos\gamma}$$

**Sezgi:** Hız vektörü, yerel yatayla $\gamma$ açısı yaparak dikey/yatay bileşenlere ayrışıyor.

> 📊 **Görselleştirme:** [→ r_dot/s_dot Kinematik HTML](Appendixes/wang_song_2018_kinematik.html)

## 2.4 İvme Vektörünün Ayrışımı — Teğetsel ve Normal İvme

Newton 2. Yasası uygulamadan önce, **ivme vektörünün** hangi bileşenlere ayrıştığını türetmemiz gerekiyor.

**Adım 1 — Hız vektörünün zaman türevi (çarpım kuralı):**

$\vec{V}=V\hat{e}_t$ ifadesinde **hem $V$ hem $\hat{e}_t$ zamana bağlı** — çarpım kuralı zorunlu:

$$\vec{a}=\frac{d\vec{V}}{dt}=\frac{d(V\hat{e}_t)}{dt}=\dot{V}\hat{e}_t+V\dot{\hat{e}}_t$$

**Adım 2 — Kritik adım: $\dot{\hat{e}}_t=\dot\gamma\hat{e}_n$'i türet.** Zincir kuralıyla ($\hat{e}_t$, $\gamma$'ya bağlı; $\gamma$ zamana bağlı):

$$\dot{\hat{e}}_t=\frac{d\hat{e}_t}{d\gamma}\cdot\frac{d\gamma}{dt}=\begin{pmatrix}\cos\gamma\\-\sin\gamma\end{pmatrix}\dot\gamma=\hat{e}_n\dot\gamma$$

> **Geometrik anlam:** Birim vektörün **uzunluğu değişemez**, sadece yönü döner. Birim çember üzerindeki bir noktanın hareket yönü, o noktanın 90° döndürülmüşüdür → $\hat{e}_n$. Dönme hızı $|\dot\gamma|$ kadardır.

**Adım 3 — İvmenin iki bileşeni:**

$$\boxed{\vec{a}=\underbrace{\dot{V}\hat{e}_t}_{\text{TEĞETSEL İVME}}+\underbrace{V\dot\gamma\hat{e}_n}_{\text{NORMAL İVME}}}$$

<svg width="560" height="310" viewBox="0 0 560 310" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- Origin -->
  <circle cx="150" cy="180" r="4" fill="#e2e8f0"/>
  <!-- ê_t axis: lower-right (gamma negative) -->
  <line x1="150" y1="180" x2="270" y2="240" stroke="#f97316" stroke-width="2"/>
  <polygon points="276,243 264,240 268,231" fill="#f97316"/>
  <text x="280" y="250" fill="#f97316" font-size="14" font-weight="bold" font-family="monospace">ê_t</text>
  <!-- ê_n axis: upper-right, perpendicular -->
  <line x1="150" y1="180" x2="210" y2="60" stroke="#a78bfa" stroke-width="2"/>
  <polygon points="213,54 203,62 212,66" fill="#a78bfa"/>
  <text x="216" y="56" fill="#a78bfa" font-size="14" font-weight="bold" font-family="monospace">ê_n</text>
  <!-- right angle marker -->
  <polyline points="163,187 170,173 156,166" stroke="#6b7280" stroke-width="1.2" fill="none"/>
  <!-- Tangential accel component (green, along ê_t) -->
  <line x1="150" y1="180" x2="228" y2="219" stroke="#4ade80" stroke-width="3.5"/>
  <polygon points="234,222 222,219 226,210" fill="#4ade80"/>
  <!-- Normal accel component (blue, along ê_n) -->
  <line x1="234" y1="222" x2="272" y2="146" stroke="#60a5fa" stroke-width="3.5"/>
  <polygon points="275,140 265,148 274,152" fill="#60a5fa"/>
  <!-- Total accel vector a (amber, resultant) -->
  <line x1="150" y1="180" x2="270" y2="148" stroke="#f59e0b" stroke-width="3"/>
  <polygon points="276,146 265,142 263,152" fill="#f59e0b"/>
  <text x="282" y="145" fill="#f59e0b" font-size="15" font-weight="bold" font-family="monospace">a</text>
  <!-- Component labels -->
  <text x="152" y="234" fill="#4ade80" font-size="13" font-weight="bold" font-family="monospace">V̇ ê_t</text>
  <text x="152" y="249" fill="#4ade80" font-size="10" font-family="monospace">teğetsel ivme</text>
  <text x="152" y="262" fill="#4ade80" font-size="10" font-family="monospace">(hız büyüklüğü değişimi)</text>
  <text x="252" y="188" fill="#60a5fa" font-size="13" font-weight="bold" font-family="monospace">Vγ̇ ê_n</text>
  <text x="252" y="203" fill="#60a5fa" font-size="10" font-family="monospace">normal ivme</text>
  <text x="252" y="216" fill="#60a5fa" font-size="10" font-family="monospace">(yön değişimi)</text>
  <!-- Info box -->
  <rect x="352" y="30" width="196" height="152" rx="6" fill="#111827" stroke="#374151" stroke-width="1"/>
  <text x="362" y="50" fill="#f59e0b" font-size="12" font-weight="bold" font-family="monospace">a = V̇ê_t + Vγ̇ê_n</text>
  <line x1="362" y1="58" x2="538" y2="58" stroke="#374151" stroke-width="1"/>
  <text x="362" y="76" fill="#4ade80" font-size="11" font-family="monospace">TEĞETSEL: V̇ ê_t</text>
  <text x="362" y="90" fill="#6b7280" font-size="10" font-family="monospace">  → hız BÜYÜKLÜĞÜ</text>
  <text x="362" y="103" fill="#6b7280" font-size="10" font-family="monospace">  → mV̇ = F·ê_t  (1c)</text>
  <line x1="362" y1="112" x2="538" y2="112" stroke="#374151" stroke-width="1"/>
  <text x="362" y="130" fill="#60a5fa" font-size="11" font-family="monospace">NORMAL: Vγ̇ ê_n</text>
  <text x="362" y="144" fill="#6b7280" font-size="10" font-family="monospace">  → hız YÖNÜ</text>
  <text x="362" y="157" fill="#6b7280" font-size="10" font-family="monospace">  → mVγ̇ = F·ê_n  (1d)</text>
  <line x1="362" y1="166" x2="538" y2="166" stroke="#374151" stroke-width="1"/>
  <text x="362" y="177" fill="#a78bfa" font-size="10" font-family="monospace">ê_t ⊥ ê_n  →  ayrışabilir</text>
  <!-- Bottom note -->
  <text x="14" y="296" fill="#6b7280" font-size="11" font-family="monospace">İki bileşen dik → Newton denklemi iki BAĞIMSIZ skaler denkleme ayrışır.</text>
</svg>

**Newton 2. Yasa vektörel formda:**

$$m\vec{a}=\vec{F}_{toplam}\;\Longrightarrow\;m\bigl(\dot{V}\hat{e}_t+V\dot\gamma\hat{e}_n\bigr)=\vec{F}_{toplam}$$

Bu **tek vektörel denklem**, $\hat{e}_t\perp\hat{e}_n$ olduğu için **iki bağımsız skaler denkleme** ayrışıyor — sıradaki iki bölüm.

## 2.5 $\dot{V}$ — TEĞET Eksende ($\hat{e}_t$) Kuvvet Dengesi

> **Yansıtma ekseni: $\hat{e}_t$** (hız yönü). Bu denklem, **hız büyüklüğünün** nasıl değiştiğini verir.

Newton denklemini $\hat{e}_t$ ile iç çarp:

$$m\Bigl(\dot{V}\underbrace{(\hat{e}_t\cdot\hat{e}_t)}_{=1}+V\dot\gamma\underbrace{(\hat{e}_t\cdot\hat{e}_n)}_{=0}\Bigr)=\vec{F}_{toplam}\cdot\hat{e}_t$$

$$\boxed{m\dot{V}=\vec{F}_{toplam}\cdot\hat{e}_t}\quad\text{← TEĞET EKSEN KUVVET DENGESİ}$$

**Kuvvetlerin $\hat{e}_t$ bileşenleri:**

| Kuvvet | $\hat{e}_t$ bileşeni | İşaret ($\gamma<0$, iniş) |
|---|---|---|
| Yerçekimi | $\vec{g}\cdot\hat{e}_t=-\sin\gamma/r^2$ | $>0$ → **hızlandırır** |
| İtki | $\vec{T}\cdot\hat{e}_t=-T\cos\alpha$ | $<0$ → frenler |
| Sürükleme | $\vec{D}\cdot\hat{e}_t=-D$ | $<0$ → frenler |

$$\boxed{\dot{V}=\frac{-T\cos\alpha-D}{m}-\frac{\sin\gamma}{r^2}}\tag{1c}$$

## 2.6 $\dot\gamma$ — NORMAL Eksende ($\hat{e}_n$) Kuvvet Dengesi

> **Yansıtma ekseni: $\hat{e}_n$** (hıza dik). Bu denklem, **yörüngenin ne kadar büküldüğünü** verir.

Aynı Newton denklemini $\hat{e}_n$ ile iç çarp:

$$m\Bigl(\dot{V}\underbrace{(\hat{e}_n\cdot\hat{e}_t)}_{=0}+V\dot\gamma\underbrace{(\hat{e}_n\cdot\hat{e}_n)}_{=1}\Bigr)=\vec{F}_{toplam}\cdot\hat{e}_n$$

$$\boxed{mV\dot\gamma=\vec{F}_{toplam}\cdot\hat{e}_n}\quad\text{← NORMAL EKSEN KUVVET DENGESİ}$$

> **Neden sol tarafta $V$ çarpanı var?** İvmenin normal bileşeni $V\dot\gamma$ (Adım 3'ten). Fiziksel anlamı: **aynı yanal kuvvet, yüksek hızda daha az yön değişimi üretir.**

**Kuvvetlerin $\hat{e}_n$ bileşenleri:**

| Kuvvet | $\hat{e}_n$ bileşeni | İşaret ($\alpha>0$, $\gamma<0$) |
|---|---|---|
| İtki | $\vec{T}\cdot\hat{e}_n=-T\sin\alpha$ | $<0$ → $\gamma$ azalır |
| Yerçekimi | $\vec{g}\cdot\hat{e}_n=-\cos\gamma/r^2$ | $<0$ → $\gamma$ azalır |
| Sürükleme | $\vec{D}\cdot\hat{e}_n=0$ | tamamen $\hat{e}_t$ yönünde |

$$\boxed{\dot\gamma=\frac{-T\sin\alpha}{mV}-\frac{\cos\gamma}{r^2V}}\tag{1d}$$

> **Kritik zincir:** $\alpha>0\Rightarrow-T\sin\alpha<0\Rightarrow\dot\gamma<0\Rightarrow\gamma\to-90°$ ✓ ($\gamma_0=-65°$'den hedef $\gamma_f=-90°$'ye)

> 📊 **Görselleştirme 1:** [→ Alfa Açısı Geometrisi HTML](Appendixes/wang_song_2018_alpha_geometrisi.html) — $\alpha$ referans ekseni ($-\hat{e}_t$'den saat yönü), $T\cos\alpha$/$T\sin\alpha$ ayrışımı
>
> 📊 **Görselleştirme 2:** [→ Gamma Kuvvet Dengesi HTML](Appendixes/wang_song_2018_gammadot.html) — $\hat{e}_n$ ekseninde $-T\sin\alpha$ ve $-g\cos\gamma$ bileşenlerinin dinamik gösterimi

## 2.7 $\dot{m}$ — Kütle Tüketimi ve $I_{sp}$

$$\boxed{\dot{m}=-\frac{T}{I_{sp}}}\tag{1e}$$

$$I_{sp}=\frac{v_e}{g_0}\;\Longleftrightarrow\;v_e=I_{sp}\cdot g_0\;\Longleftrightarrow\;T=v_e\cdot(-\dot{m})$$

$I_{sp}$, roket motorunun **egzoz hızını** ($v_e$) $g_0=9.81$ m/s²'ye bölünmüş hali — Tsiolkovsky'nin orijinal formülasyonuyla doğrudan bağlantılı.

## 2.8 Sürükleme — Denklem (2)

$$D=\tfrac{1}{2}\rho V^2 S_{ref}C_D,\qquad \rho=\rho_0e^{-\beta h},\qquad h=r-R_e\tag{2}$$

$D\propto V^2$: iniş başında (yüksek hız) baskın, iniş sonunda (düşük hız) ihmal edilebilir.

## 2.9 Sınır Koşulları ve Kısıtlar — Eq.(3)-(5)

$$\mathbf{x}_0=[3\text{km},0,280\text{m/s},{-65°},55000\text{kg}]^T,\quad \mathbf{x}_f=[0,1\text{km},\leq V_{safe},-90°,\geq m_{dry}]^T$$

$$V_f\leq1\text{m/s},\quad\gamma_f=-90°,\quad|\alpha_f|\leq2°,\quad m_f\geq49000\text{kg}\tag{4}$$

$$412.7\text{kN}\leq T\leq1375.6\text{kN},\qquad-10°\leq\alpha\leq+10°\tag{5}$$

> ⚠️ **Onaylı yazım hatası:** Eq.(4)'te makale $\gamma_f=90°$ yazıyor; fiziksel doğrusu $-90°$ (§4.2 metni "*can achieve -90°*" ve Tablo 2'deki $\gamma_0=-65°$ ile doğrulandı — negatif açı sistemi kullanılıyor).

> ⚠️ **Nonkonveksite kaynağı:** $T_{min}>0$ → izin verilen küme $\{0\}\cup[T_{min},T_{max}]$ **dışbükey değil** (motor ya kapalı ya bu aralıkta). §2.3'te relaxation ile çözülecek.

---

# 3. Matematiksel Formülasyon — §2.2 Performans İndeksi

$$J=-m_f\tag{6}$$

$m_0$ (başlangıç kütle) sabit olduğundan $\min(m_0-m_f)\equiv\min(-m_f)$ — yakıt minimizasyonu ile terminal kütle maksimizasyonu **matematiksel olarak eşdeğer**, ama $m_f$ zaten bir durum değişkeni olduğundan doğrudan kullanmak hesaplama açısından en verimli.

$$Problem0:\quad\min J=-m_f\quad\text{s.t. Eqs.}(1)(3)(4)(5)\tag{7}$$

"Problem0" adı, bunun **henüz dışbükeyleştirilmemiş** orijinal problem olduğunu vurguluyor.

---

# 4. Karşılaşılan Zorluk 1: Nonlineerlik + Serbest Zaman → §2.3 Convexification

## 4.1 Neden Problem0 Doğrudan Çözülemez

> *"The fuel consumption for rocket landing is related to the flight time... makes Problem0 be a terminal time free optimization problem, which does not apply to convex optimization."*

İki engel: **(1)** $t_f$ bilinmiyor — konveks optimizasyon sabit zaman ufku ister. **(2)** dinamikler nonlineer + $T_{min}>0$ kısıtı nonkonveks.

## 4.2 Çözüm A — Zaman Normalizasyonu (Eq.8)

$$\frac{t-t_0}{t_f-t_0}=\frac{\tau-0}{1-0}=\tau\tag{8}$$

**Bu bir NORMALİZASYON işlemidir:** Bilinmeyen uzunluktaki $[t_0,t_f]$ aralığı, **sabit** $[0,1]$ aralığına eşleniyor. Gerçek zamandaki herhangi bir $t$ anı, $[0,1]$ arasında bir $\tau$ değerine düşüyor.

<svg width="560" height="260" viewBox="0 0 560 260" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- REAL TIME AXIS (purple, top) -->
  <line x1="70" y1="60" x2="500" y2="60" stroke="#c084fc" stroke-width="3"/>
  <polygon points="508,60 496,55 496,65" fill="#c084fc"/>
  <!-- tick marks on real axis -->
  <line x1="70" y1="45" x2="70" y2="75" stroke="#c084fc" stroke-width="3"/>
  <line x1="285" y1="48" x2="285" y2="72" stroke="#c084fc" stroke-width="2"/>
  <line x1="480" y1="45" x2="480" y2="75" stroke="#c084fc" stroke-width="3"/>
  <text x="58" y="38" fill="#c084fc" font-size="15" font-weight="bold" font-family="monospace">t₀</text>
  <text x="272" y="38" fill="#c084fc" font-size="14" font-family="monospace">t</text>
  <text x="468" y="38" fill="#c084fc" font-size="15" font-weight="bold" font-family="monospace">t_f</text>
  <text x="516" y="65" fill="#c084fc" font-size="12" font-family="monospace">gerçek</text>
  <text x="516" y="79" fill="#c084fc" font-size="12" font-family="monospace">zaman</text>
  <!-- BILINMIYOR label -->
  <text x="330" y="30" fill="#f87171" font-size="11" font-family="monospace">t_f BİLİNMİYOR!</text>
  <!-- NORMALIZED AXIS (green, bottom) -->
  <line x1="70" y1="185" x2="285" y2="185" stroke="#4ade80" stroke-width="3"/>
  <line x1="70" y1="170" x2="70" y2="200" stroke="#4ade80" stroke-width="3"/>
  <line x1="285" y1="170" x2="285" y2="200" stroke="#4ade80" stroke-width="3"/>
  <line x1="180" y1="174" x2="180" y2="196" stroke="#4ade80" stroke-width="2"/>
  <text x="60" y="222" fill="#4ade80" font-size="17" font-weight="bold" font-family="monospace">0</text>
  <text x="170" y="222" fill="#4ade80" font-size="14" font-family="monospace">τ</text>
  <text x="278" y="222" fill="#4ade80" font-size="17" font-weight="bold" font-family="monospace">1</text>
  <text x="304" y="190" fill="#4ade80" font-size="12" font-family="monospace">normalize (SABİT)</text>
  <!-- MAPPING ARROWS (curved, black/gray) -->
  <path d="M 70 78 Q 62 130 68 168" stroke="#94a3b8" stroke-width="2" fill="none"/>
  <polygon points="69,175 64,164 74,164" fill="#94a3b8"/>
  <path d="M 283 78 Q 250 130 184 168" stroke="#94a3b8" stroke-width="2" fill="none"/>
  <polygon points="179,172 190,166 192,176" fill="#94a3b8"/>
  <path d="M 478 78 Q 430 140 291 172" stroke="#94a3b8" stroke-width="2" fill="none"/>
  <polygon points="285,174 296,167 299,177" fill="#94a3b8"/>
  <!-- Formula box -->
  <rect x="340" y="205" width="212" height="46" rx="6" fill="#111827" stroke="#374151" stroke-width="1"/>
  <text x="350" y="224" fill="#e2e8f0" font-size="12" font-family="monospace">τ = (t − t₀)/(t_f − t₀)</text>
  <text x="350" y="242" fill="#6b7280" font-size="11" font-family="monospace">t = t₀ + τ(t_f − t₀)</text>
  <!-- Bottom-left note -->
  <text x="14" y="248" fill="#6b7280" font-size="11" font-family="monospace">t_f artık KONTROL DEĞİŞKENİ</text>
</svg>

**Sonuç:** $t_f$ artık bir **kontrol değişkeni** — optimizasyon algoritması onu da $u_1,u_2,u_3$ ile birlikte optimize ediyor. Konveks optimizasyonun "sabit zaman ufku" kısıtlaması bu şekilde aşılıyor.

## 4.3 Zincir Kuralı — Neden Her Denklem $t_f$ ile Çarpılıyor? (Eq.9)

Dinamik denklemler artık $t$'ye değil, **$\tau$'ya göre** türetilecek. Zincir kuralı:

$$\frac{dx}{d\tau}=\frac{dx}{dt}\cdot\frac{dt}{d\tau}$$

**$dt/d\tau$'yu hesaplayalım.** Eq.(8)'den $t$'yi çözelim:

$$t-t_0=\tau(t_f-t_0)\;\Longrightarrow\;t=t_0+\tau(t_f-t_0)$$

$\tau$'ya göre türev:

$$\boxed{\frac{dt}{d\tau}=t_f-t_0}$$

> ⚠️ **Genel form vs. makalenin kullanımı:** Genel türetim $t_f-t_0$ verir. Makale Eq.(9)'da doğrudan $t_f$ kullanıyor — bu, **$t_0=0$ zımni varsayımıyla** tutarlı (iniş fazının başlangıcı referans alınıyor). MPC'de $t_0$ her iterasyonda güncellendiği için, kendi implementasyonunuzda **genel formu** ($t_f-t_0$) kullanmanız gerekebilir.

**Dönüşümün her denkleme uygulanması:**

| Orijinal (Eq.1, $d/dt$) | Dönüşmüş (Eq.9, $d/d\tau$) |
|---|---|
| $\dot{r}=V\sin\gamma$ | $\dot{r}=t_f\,V\sin\gamma$ |
| $\dot{s}=V\cos\gamma$ | $\dot{s}=t_f\,V\cos\gamma$ |
| $\dot{V}=\frac{-T\cos\alpha-D}{m}-\frac{\sin\gamma}{r^2}$ | $\dot{V}=t_f\Bigl(\frac{-T\cos\alpha-D}{m}-\frac{\sin\gamma}{r^2}\Bigr)$ |
| $\dot\gamma=\frac{-T\sin\alpha}{mV}-\frac{\cos\gamma}{r^2V}$ | $\dot\gamma=t_f\Bigl(\frac{-T\sin\alpha}{mV}-\frac{\cos\gamma}{r^2V}\Bigr)$ |
| $\dot{m}=-T/I_{sp}$ | $\dot{m}=-t_f\,T/I_{sp}$ |
| — | **$\dot{t}=t_f$** ← YENİ durum denklemi |

> **Notasyon uyarısı:** Eq.(9)'dan itibaren üstteki nokta artık $d/dt$ değil, **$d/d\tau$** anlamına geliyor.

**Neden $\dot{t}=t_f$ eklendi?** Gerçek zaman $t$, artık bir **durum değişkeni**. Kendi türevi de zincir kuralından geliyor: $\frac{dt}{d\tau}=t_f$. Bu sayede optimizasyon çözümünde her $\tau$ noktasının gerçek zaman karşılığı da hesaplanmış oluyor.

**Sezgi — neden ölçekleme mantıklı?** $\tau=0.5$ noktasındasınız (yolun yarısı). Gerçek zamanda nerede olduğunuz $t_f$'ye bağlı: $t_f=20s$ ise 10. saniyedesiniz, $t_f=10s$ ise 5. saniyedesiniz. Dolayısıyla **birim $\tau$ başına** kat edilen gerçek mesafe/hız değişimi, $t_f$ ile orantılı.

$$\dot{r}=t_fV\sin\gamma,\;\ldots,\;\dot{t}=t_f\tag{9}$$

## 4.4 Çözüm B — Değişken Dönüşümü (Eq.10-11)

$$z=\ln(m),\qquad u_1=T\cos\alpha/m,\quad u_2=T\sin\alpha/m,\quad u_3=T/m\tag{10}$$

**Neden bu dönüşüm?** $T\cos\alpha/m$ gibi çarpımsal/trigonometrik terimler, yeni değişkenlerle **doğrudan doğrusal** hale geliyor. Bedeli: yeni bir kısıt gerekiyor —

$$u_1^2+u_2^2=u_3^2\tag{11}$$

(Pisagor özdeşliğinden: $(T\cos\alpha)^2+(T\sin\alpha)^2=T^2$.) **Bu eşitlik henüz nonkonveks** — bir koninin sadece yüzeyi.

$z=\ln m$ dönüşümünün nedeni: $u_3=T/m$ ile $\dot{m}=-T/I_{sp}$ birleşince $u_3\cdot m$ çarpımı (nonlineer) ortaya çıkardı; $\dot{z}=\dot{m}/m=-u_3/I_{sp}$ ise **tamamen doğrusal.**

## 4.5 Kısıtların Yeni Değişkenlere Dönüşümü (Eq.12-14)

### 4.5.1 Yeni Amaç Fonksiyonu

$$J=-z_f\tag{12}$$

$z=\ln m$ monoton artan olduğundan $\max m_f\equiv\max z_f\equiv\min(-z_f)$ — **optimal çözüm değişmiyor.**

### 4.5.2 TERMİNAL Kısıtların Dönüşümü (Eq.4 → Eq.13)

Orijinal terminal kısıtlar (§2.9, Eq.4): $\;\|\alpha_f\|\leq\alpha_{safe}$ ve $m_f\geq m_{dry}$

**Dönüşüm 1 — Hücum açısı kısıtı:**

$u_1,u_2$ tanımlarından oranı al:
$$\frac{u_2}{u_1}=\frac{T\sin\alpha/m}{T\cos\alpha/m}=\frac{\sin\alpha}{\cos\alpha}=\tan\alpha$$

$\tan$ fonksiyonu $[-10°,+10°]$ aralığında monoton artan olduğundan:
$$\|\alpha_f\|\leq\alpha_{safe}\;\Longrightarrow\;\left\|\frac{u_2}{u_1}\right\|_{t_f}\leq\tan(\alpha_{safe})$$

**Dönüşüm 2 — Kütle kısıtı:**

$z=\ln m$ monoton artan → $m_f\geq m_{dry}\Leftrightarrow z_f\geq z_{dry}$ (burada $z_{dry}=\ln(m_{dry})$). Makale bunu üstel formda yazıyor:
$$e^{z}\geq e^{z_{dry}}$$

$$\boxed{\left\|\frac{u_2}{u_1}\right\|_{t_f}\leq\tan(\alpha_{safe}),\qquad e^z\geq e^{z_{dry}}}\tag{13}$$

| Orijinal (Eq.4) | Dönüşmüş (Eq.13) | Kullanılan özdeşlik |
|---|---|---|
| $\|\alpha_f\|\leq\alpha_{safe}$ | $\|u_2/u_1\|_{t_f}\leq\tan\alpha_{safe}$ | $u_2/u_1=\tan\alpha$ |
| $m_f\geq m_{dry}$ | $e^{z}\geq e^{z_{dry}}$ | $z=\ln m$, $z_{dry}=\ln m_{dry}$ |
| $V_f\leq V_{safe}$, $\gamma_f=-90°$ | **değişmedi** (Eq.3'te kaldı) | $V,\gamma$ dönüştürülmedi |

### 4.5.3 SÜREÇ Kısıtlarının Dönüşümü (Eq.5 → Eq.14)

Orijinal süreç kısıtları (§2.9, Eq.5): $\;T_{min}\leq T\leq T_{max}$ ve $\alpha_{min}\leq\alpha\leq\alpha_{max}$

**Dönüşüm 1 — İtki kısıtı, adım adım:**

$u_3=T/m$ tanımından $T=u_3\cdot m$. $z=\ln m\Rightarrow m=e^z$, yani:
$$T=u_3\,e^z$$

Bunu $T_{min}\leq T\leq T_{max}$'a koy:
$$T_{min}\leq u_3e^z\leq T_{max}$$

Her tarafı $e^z>0$'a böl (pozitif sayıya bölme, eşitsizlik yönünü **değiştirmez**):
$$\frac{T_{min}}{e^z}\leq u_3\leq\frac{T_{max}}{e^z}\;\Longrightarrow\;T_{min}e^{-z}\leq u_3\leq T_{max}e^{-z}$$

**Dönüşüm 2 — Hücum açısı süreç kısıtı:** Eq.13'teki aynı $\tan\alpha=u_2/u_1$ özdeşliğiyle.

$$\boxed{T_{min}e^{-z}\leq u_3\leq T_{max}e^{-z},\qquad\tan\alpha_{min}\leq\frac{u_2}{u_1}\leq\tan\alpha_{max}}\tag{14}$$

> ⚠️ **$\alpha_{safe}$ vs $\alpha_{min}/\alpha_{max}$ — İKİ FARKLI KISIT:**
> - $\alpha_{safe}=2°$ → **sadece** $t=t_f$ anında geçerli (Eq.13, terminal)
> - $[\alpha_{min},\alpha_{max}]=[-10°,+10°]$ → **tüm uçuş boyunca** geçerli (Eq.14, süreç)
>
> $\alpha_{safe}$ çok daha sıkı çünkü iniş anında roket neredeyse tam dik olmalı.

> ⚠️ **Kalan nonlineerlik:** $e^{-z}$ terimi hâlâ nonlineer — Eq.(21)'de successive linearization ile doğrusallaştırılacak.

### 4.5.4 Sadeleştirilmiş Dinamikler (Eq.15)

$u_1,u_2,u_3,z$ tanımları Eq.(9)'a yerleştirilince:

$$\dot{r}=t_fV\sin\gamma,\;\dot{V}=t_f(-u_1-D/m-\sin\gamma/r^2),\;\dot\gamma=t_f(-u_2/V-\cos\gamma/r^2V),\;\dot{z}=-t_fu_3/I_{sp}\tag{15}$$

| Eq.(9)'daki terim | Eq.(15)'te yerine geçen | Neden |
|---|---|---|
| $-T\cos\alpha/m$ | $-u_1$ | $u_1$ tanımı doğrudan |
| $-T\sin\alpha/(mV)$ | $-u_2/V$ | $u_2$ tanımı doğrudan |
| $\dot{m}=-t_fT/I_{sp}$ | $\dot{z}=-t_fu_3/I_{sp}$ | $z=\ln m$ + $u_3=T/m$ |

$$Problem1:\;\min\;Eq.(12)\;\text{s.t.}\;Eqs.(3)(11)(13)(14)(15)\tag{16}$$

**Problem0 → Problem1 Karşılaştırma Tablosu:**

| Bileşen | Problem0 (Eq.7) | Problem1 (Eq.16) | Ne Değişti? |
|---|---|---|---|
| **Amaç** | $J=-m_f$ (Eq.6) | $J=-z_f$ (Eq.12) | $z=\ln m$ dönüşümü |
| **Dinamikler** | Eq.(1) — $T,\alpha$ ile | Eq.(15) — $u_1,u_2,u_3,z$ ile | Değişken dönüşümü |
| **Sınır koşulları** | Eq.(3) | Eq.(3) — **aynı** | Değişmedi |
| **Terminal kısıt** | Eq.(4) — $\|\alpha_f\|\leq\alpha_{safe}$ | Eq.(13) — $\|u_2/u_1\|\leq\tan\alpha_{safe}$ | Yeni değişkenlerle |
| **Süreç kısıtı** | Eq.(5) — $T_{min}\leq T\leq T_{max}$ | Eq.(14) — $T_{min}e^{-z}\leq u_3\leq T_{max}e^{-z}$ | Yeni değişkenlerle |
| **Ek kısıt** | — | **Eq.(11)** $u_1^2+u_2^2=u_3^2$ | Yeni (henüz nonkonveks!) |

> **Durum:** $t_f$ artık kontrol değişkeni (serbest zaman sorunu çözüldü). Ama dinamikler (Eq.15) hâlâ nonlineer ($t_f\cdot(\ldots)$ çarpımları, $1/r^2$, $1/V$), Eq.(11) hâlâ nonkonveks. → Successive linearization + relaxation gerekli.

## 4.6 Çözüm C — Successive Linearization (Eq.17-21)

**Genel Taylor açılımı mantığı — adım adım $Ax+Bu+C$'ye:**

**Adım 1:** $f(x,u)$'yu $(x^k,u^k)$ etrafında birinci dereceden Taylor ile aç:
$$f(x,u)\approx f(x^k,u^k)+\underbrace{\frac{\partial f}{\partial x}\Big|_{x^k,u^k}}_{:=A}(x-x^k)+\underbrace{\frac{\partial f}{\partial u}\Big|_{x^k,u^k}}_{:=B}(u-u^k)$$

**Adım 2:** Parantezleri aç:
$$f(x,u)\approx f(x^k,u^k)+Ax-Ax^k+Bu-Bu^k$$

**Adım 3:** $x,u$ içermeyen her terimi $C$'de topla:
$$\underbrace{f(x^k,u^k)-Ax^k-Bu^k}_{:=C}$$

**Adım 4:** Sonuç:
$$\boxed{\dot{x}=Ax+Bu+C}\tag{17}$$

$A,B$ **sabit matrisler**: değerleri bir önceki iterasyonun çözümü $(x^k,u^k)$'dan hesaplanan sayılar — değişken değil. Bu yüzden Eq.(17) $x,u$ cinsinden **doğrusal (affine).**

$$C(x^k,u^k)=\dot{x}(x^k,u^k)-A(x^k,u^k)x^k-B(x^k,u^k)u^k\tag{20}$$

$x=[r,s,V,\gamma,z,t]^T$ ($6\times1$), $u=[u_1,u_2,u_3,t_f]^T$ ($4\times1$) → $A\in\mathbb{R}^{6\times6}$, $B\in\mathbb{R}^{6\times4}$.

**Aynı mantık, $e^{-z}$'ye özel uygulama (Eq.21):**

$$e^{-z}\approx e^{-z^k}-e^{-z^k}(z-z^k)\tag{21}$$

(Eq.14'teki nonlineer $e^{-z}$ terimini, mevcut iterasyonun $z^k$ değeri etrafında teğet çizgiyle yaklaştırır.)

**Successive linearization döngüsü:**
```
k=0: kaba tahmin x⁰ → doğrusallaştır → çöz → x¹
k=1: x¹ etrafında YENİDEN doğrusallaştır → çöz → x²
...  |x^(k+1)-x^k| < ε olana kadar tekrarla (Eq.24)
```
Bu, Introduction'daki **SCP (Sequential Convex Programming)** kavramının doğrudan uygulaması.

## 4.7 Çözüm D — Relaxation (Eq.22) — ASIL "Gevşetme" Burada

$$\boxed{u_1^2+u_2^2\leq u_3^2}\tag{22}$$

Eq.(11)'deki **eşitlik**, burada **eşitsizliğe** gevşetiliyor.

<svg width="560" height="290" viewBox="0 0 560 290" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- LEFT: cone SURFACE only (nonconvex) -->
  <text x="30" y="28" fill="#f87171" font-size="13" font-weight="bold" font-family="monospace">Eq.(11): u₁²+u₂² = u₃²</text>
  <text x="30" y="45" fill="#f87171" font-size="11" font-family="monospace">Sadece KONİ YÜZEYİ</text>
  <text x="30" y="60" fill="#f87171" font-size="12" font-weight="bold" font-family="monospace">→ NONKONVEKS ✗</text>
  <!-- cone outline left: apex at (135,215), opening upward -->
  <line x1="135" y1="215" x2="75" y2="95" stroke="#f87171" stroke-width="2.5"/>
  <line x1="135" y1="215" x2="195" y2="95" stroke="#f87171" stroke-width="2.5"/>
  <ellipse cx="135" cy="95" rx="60" ry="17" fill="none" stroke="#f87171" stroke-width="2.5"/>
  <!-- u3 axis -->
  <line x1="135" y1="215" x2="135" y2="78" stroke="#6b7280" stroke-width="1" stroke-dasharray="4,3"/>
  <text x="140" y="80" fill="#6b7280" font-size="11" font-family="monospace">u₃</text>
  <circle cx="135" cy="215" r="3" fill="#e2e8f0"/>
  <!-- two points ON surface + connecting line going INSIDE -->
  <circle cx="88" cy="125" r="4" fill="#fbbf24"/>
  <circle cx="182" cy="125" r="4" fill="#fbbf24"/>
  <line x1="88" y1="125" x2="182" y2="125" stroke="#fbbf24" stroke-width="2" stroke-dasharray="4,3"/>
  <text x="76" y="150" fill="#fbbf24" font-size="10" font-family="monospace">doğru parçası</text>
  <text x="70" y="163" fill="#fbbf24" font-size="10" font-family="monospace">yüzeyin DIŞINDA</text>
  <text x="30" y="248" fill="#6b7280" font-size="10" font-family="monospace">İçi BOŞ → konveks değil</text>
  <!-- ARROW -->
  <line x1="240" y1="150" x2="310" y2="150" stroke="#4ade80" stroke-width="3"/>
  <polygon points="318,150 305,144 305,156" fill="#4ade80"/>
  <text x="238" y="138" fill="#4ade80" font-size="11" font-weight="bold" font-family="monospace">RELAXATION</text>
  <text x="248" y="172" fill="#4ade80" font-size="10" font-family="monospace">(kayıpsız)</text>
  <!-- RIGHT: FILLED cone (convex) -->
  <text x="350" y="28" fill="#4ade80" font-size="13" font-weight="bold" font-family="monospace">Eq.(22): u₁²+u₂² ≤ u₃²</text>
  <text x="350" y="45" fill="#4ade80" font-size="11" font-family="monospace">DOLU KONİ (iç + yüzey)</text>
  <text x="350" y="60" fill="#4ade80" font-size="12" font-weight="bold" font-family="monospace">→ KONVEKS ✓ (SOC)</text>
  <!-- filled cone: apex (450,215) -->
  <path d="M 450 215 L 390 95 A 60 17 0 0 0 510 95 Z" fill="#4ade80" fill-opacity="0.22" stroke="#4ade80" stroke-width="2.5"/>
  <ellipse cx="450" cy="95" rx="60" ry="17" fill="#4ade80" fill-opacity="0.35" stroke="#4ade80" stroke-width="2.5"/>
  <line x1="450" y1="215" x2="450" y2="78" stroke="#6b7280" stroke-width="1" stroke-dasharray="4,3"/>
  <text x="455" y="80" fill="#6b7280" font-size="11" font-family="monospace">u₃</text>
  <circle cx="450" cy="215" r="3" fill="#e2e8f0"/>
  <!-- two points + connecting line INSIDE the solid -->
  <circle cx="403" cy="125" r="4" fill="#fbbf24"/>
  <circle cx="497" cy="125" r="4" fill="#fbbf24"/>
  <line x1="403" y1="125" x2="497" y2="125" stroke="#fbbf24" stroke-width="2.5"/>
  <text x="392" y="150" fill="#fbbf24" font-size="10" font-family="monospace">doğru parçası</text>
  <text x="400" y="163" fill="#fbbf24" font-size="10" font-family="monospace">İÇERİDE kalıyor ✓</text>
  <text x="350" y="248" fill="#6b7280" font-size="10" font-family="monospace">İçi DOLU → konveks</text>
  <!-- Bottom note -->
  <line x1="20" y1="262" x2="540" y2="262" stroke="#374151" stroke-width="1"/>
  <text x="20" y="280" fill="#a78bfa" font-size="11" font-family="monospace">Konveks küme tanımı: kümedeki 2 noktayı birleştiren doğru parçası TAMAMEN kümede kalmalı.</text>
</svg>

**Konveks küme tanımı üzerinden kanıt:** Bir küme konvekstir ⟺ içindeki herhangi iki noktayı birleştiren doğru parçası tamamen kümede kalır.

- **Koni yüzeyi (Eq.11):** Yüzeydeki iki noktayı birleştiren doğru, koninin **içine** düşer — yüzeyde kalmaz → **konveks değil**
- **Dolu koni (Eq.22):** İçindeki iki noktayı birleştiren doğru, yine içeride kalır → **konveks** ✓

**Second-Order Cone (SOC) nedir?** $\|x\|_2\leq t$ formundaki kısıt — "second-order" ismi, koninin şeklinden değil, **tanımlayan ifadenin derecesinden** (karesel terimler) geliyor. Eşitlik = sadece koninin yüzeyi (**nonkonveks**); eşitsizlik = dolu koni (**konveks**, çünkü içindeki iki noktayı birleştiren doğru parçası hep içeride kalır).

**Kayıpsızlık (lossless):** Optimal çözümde bu eşitsizlik daima sınırda aktif olur ($u_1^2+u_2^2=u_3^2$) — Liu [11]'e referansla kanıtlanmış bir sonuç. *(Not: Bu tez için 6-DOF'ta ayrıca kanıtlanmalı/literatürden desteklenmeli — bkz. §7.4)*

$$Problem2:\;\min\;Eq.(12)\;\text{s.t.}\;Eqs.(3)(13)(14)(17)(21)(22)\tag{23}$$

**Problem2 artık tam konveks** — bir **SOCP** (Second-Order Cone Program).

### Problem1 → Problem2 Karşılaştırma Tablosu

| Bileşen | Problem1 (Eq.16) | Problem2 (Eq.23) | Ne Değişti? |
|---|---|---|---|
| **Amaç** | $J=-z_f$ (Eq.12) | $J=-z_f$ (Eq.12) | **aynı** |
| **Sınır koşulları** | Eq.(3) | Eq.(3) | **aynı** |
| **Terminal kısıt** | Eq.(13) | Eq.(13) | **aynı** |
| **Süreç kısıtı** | Eq.(14) — $e^{-z}$ nonlineer | Eq.(14) **+ Eq.(21)** | $e^{-z}$ doğrusallaştırıldı |
| **Dinamikler** | Eq.(15) — **nonlineer** | **Eq.(17)** — $\dot{x}=Ax+Bu+C$ | Successive linearization |
| **Kontrol kısıtı** | Eq.(11) — $u_1^2+u_2^2=u_3^2$ **nonkonveks** | **Eq.(22)** — $u_1^2+u_2^2\leq u_3^2$ | SOC relaxation |

### Tam Dönüşüm Akışı (Makalenin Kendi Sırası)

<svg width="580" height="400" viewBox="0 0 580 400" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- PROBLEM 0 box -->
  <rect x="30" y="20" width="220" height="76" rx="8" fill="#7f1d1d" fill-opacity="0.3" stroke="#f87171" stroke-width="2"/>
  <text x="42" y="42" fill="#f87171" font-size="14" font-weight="bold" font-family="monospace">Problem0  (Eq.7)</text>
  <text x="42" y="60" fill="#e2e8f0" font-size="11" font-family="monospace">min J = −m_f</text>
  <text x="42" y="76" fill="#e2e8f0" font-size="11" font-family="monospace">s.t. Eq.(1)(3)(4)(5)</text>
  <text x="42" y="90" fill="#fca5a5" font-size="10" font-family="monospace">NONLİNEER + NONKONVEKS</text>
  <!-- Arrow 1 -->
  <line x1="140" y1="98" x2="140" y2="150" stroke="#94a3b8" stroke-width="2.5"/>
  <polygon points="140,158 134,146 146,146" fill="#94a3b8"/>
  <!-- Transformations 1 -->
  <rect x="266" y="98" width="290" height="60" rx="6" fill="#111827" stroke="#374151" stroke-width="1"/>
  <text x="276" y="116" fill="#c084fc" font-size="11" font-family="monospace">① Zaman normalizasyonu (Eq.8-9)</text>
  <text x="286" y="130" fill="#6b7280" font-size="10" font-family="monospace">[t₀,t_f]→[0,1],  t_f = kontrol değişkeni</text>
  <text x="276" y="146" fill="#c084fc" font-size="11" font-family="monospace">② Değişken dönüşümü (Eq.10)</text>
  <text x="286" y="156" fill="#6b7280" font-size="10" font-family="monospace">z=ln m,  u₁,u₂,u₃  →  Eq.(11) kısıtı doğar</text>
  <!-- PROBLEM 1 box -->
  <rect x="30" y="160" width="220" height="86" rx="8" fill="#78350f" fill-opacity="0.3" stroke="#fbbf24" stroke-width="2"/>
  <text x="42" y="182" fill="#fbbf24" font-size="14" font-weight="bold" font-family="monospace">Problem1  (Eq.16)</text>
  <text x="42" y="200" fill="#e2e8f0" font-size="11" font-family="monospace">min J = −z_f</text>
  <text x="42" y="216" fill="#e2e8f0" font-size="11" font-family="monospace">s.t. Eq.(3)(11)(13)(14)(15)</text>
  <text x="42" y="230" fill="#fcd34d" font-size="10" font-family="monospace">t_f serbest ✓</text>
  <text x="42" y="242" fill="#fcd34d" font-size="10" font-family="monospace">ama hâlâ NONLİNEER + NONKONVEKS</text>
  <!-- Arrow 2 -->
  <line x1="140" y1="248" x2="140" y2="300" stroke="#94a3b8" stroke-width="2.5"/>
  <polygon points="140,308 134,296 146,296" fill="#94a3b8"/>
  <!-- Transformations 2 -->
  <rect x="266" y="248" width="290" height="60" rx="6" fill="#111827" stroke="#374151" stroke-width="1"/>
  <text x="276" y="266" fill="#4ade80" font-size="11" font-family="monospace">③ Successive linearization (Eq.17-21)</text>
  <text x="286" y="280" fill="#6b7280" font-size="10" font-family="monospace">Eq.(15)→Eq.(17): ẋ=Ax+Bu+C,  e⁻ᶻ→Eq.(21)</text>
  <text x="276" y="296" fill="#4ade80" font-size="11" font-family="monospace">④ SOC Relaxation (Eq.22)</text>
  <text x="286" y="306" fill="#6b7280" font-size="10" font-family="monospace">Eq.(11) eşitlik → eşitsizlik (kayıpsız)</text>
  <!-- PROBLEM 2 box -->
  <rect x="30" y="310" width="220" height="76" rx="8" fill="#14532d" fill-opacity="0.35" stroke="#4ade80" stroke-width="2.5"/>
  <text x="42" y="332" fill="#4ade80" font-size="14" font-weight="bold" font-family="monospace">Problem2  (Eq.23)</text>
  <text x="42" y="350" fill="#e2e8f0" font-size="11" font-family="monospace">min J = −z_f</text>
  <text x="42" y="366" fill="#e2e8f0" font-size="10" font-family="monospace">s.t. Eq.(3)(13)(14)(17)(21)(22)</text>
  <text x="42" y="380" fill="#86efac" font-size="11" font-weight="bold" font-family="monospace">TAM KONVEKS (SOCP) ✓</text>
  <!-- Right note for Problem2 -->
  <rect x="266" y="318" width="290" height="64" rx="6" fill="#111827" stroke="#4ade80" stroke-width="1"/>
  <text x="276" y="338" fill="#4ade80" font-size="11" font-weight="bold" font-family="monospace">Primal-dual interior point</text>
  <text x="276" y="354" fill="#6b7280" font-size="10" font-family="monospace">→ polynomial zamanda çözülür</text>
  <text x="276" y="368" fill="#6b7280" font-size="10" font-family="monospace">→ MPC'nin her iterasyonunda çözülen</text>
  <text x="276" y="380" fill="#6b7280" font-size="10" font-family="monospace">   problem (Section 3)</text>
</svg>

## 4.8 Çözüm Yöntemi — Primal-Dual Interior Point Method

**Polynomial zaman:** $T(n)=O(n^k)$ formunda büyüme — exponential ($O(2^n)$) büyümenin aksine, pratik olarak çözülebilir. Makale sadece "polynomial time" diyor, kesin üs vermiyor.

**Interior Point vs Simplex — Geometrik Karşılaştırma:**

```
Simplex Metodu:                    Interior Point Metodu:
Feasible bölgenin                  Feasible bölgenin
KÖŞE noktalarında dolaşır          İÇİNDEN başlar, sınıra doğru ilerler

    ●───●───●                          ●───●───●
    |       |                          |  •→•→ |
    ●   ●   ●  ← köşe köşe            |    ↓  |
    |       |     ilerler              ●───●───●
    ●───●───●                              ↓ optimal

Sonuç: çok iterasyon                Sonuç: daha az iterasyon
```

**Primal-dual:** hem orijinal problemi (primal) hem matematiksel "ikizini" (dual) aynı anda çözer — optimal olduğu anında doğrulanabilir ($x^*$primal = $\lambda^*$dual → duality gap = 0).

$$\max_i|x_i^{k+1}-x_i^k|\leq\varepsilon_x,\quad\max_i|u_i^{k+1}-u_i^k|\leq\varepsilon_u\tag{24}$$

---

# 5. Karşılaşılan Zorluk 2: Gerçek Zamanlı Güdüm → Section 3 MPC

## 5.1 Neden Klasik MPC Yetersiz

**İki zaman ölçeği çarpışması:** Güdüm döngüsü hızlı olmalı (örn. 0.1s), ama yörünge optimizasyonu hesaplama hızı için seyrek nokta kullanıyor (21 nokta / 20s = 1s aralık).

**Lineer interpolasyon çözümü başarısız:** Ara noktalar düz çizgiyle dolduruluyor ama **süreç kısıtları ara noktalarda hiç kontrol edilmiyor** — kontrolün keskin değiştiği ("turning point") bölgelerde özellikle sorunlu.

**Klasik takip-tabanlı MPC (Eq.25) da kısmi çözüm:**

$$J_j=\sum_{k=1}^{N_p}(\Delta x^TQ\Delta x+\Delta u^TR\Delta u),\quad\Delta x=x(j+k|j)-x_{ref}(j+k|j)\tag{25}$$

Kısıtları optimizasyon içine koyarak sağlıyor, ama referans hâlâ seyrek olduğundan **turning point'in tam konumunu** hassas yakalayamıyor.

> **Not:** Eq.(25) makalede sadece **karşılaştırma amaçlı** — Wang & Song bunu **kullanmıyor.**

## 5.2 Çözüm — Novel Receding Horizon Stratejisi

**İki değişiklik:**

**(1) Ayrık nokta konumu:** Her güdüm döngüsünde, mevcut an ($t_0$) ile bir sonraki güdüm anı ($t_1$) arasına **ekstra bir nokta** ekleniyor. Diğer noktalar referans yörüngedeki yerlerinde kalıyor:

<svg width="580" height="270" viewBox="0 0 580 270" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- Row labels -->
  <text x="10" y="44" fill="#9ca3af" font-size="11" font-family="monospace">Referans</text>
  <text x="10" y="57" fill="#9ca3af" font-size="11" font-family="monospace">Yörünge</text>
  <text x="10" y="106" fill="#9ca3af" font-size="11" font-family="monospace">MPC 1</text>
  <text x="10" y="156" fill="#9ca3af" font-size="11" font-family="monospace">MPC 2</text>
  <text x="10" y="206" fill="#9ca3af" font-size="11" font-family="monospace">MPC k</text>

  <!-- ROW 1: Reference trajectory -->
  <line x1="95" y1="50" x2="530" y2="50" stroke="#6b7280" stroke-width="1.5"/>
  <circle cx="95" cy="50" r="5" fill="#94a3b8"/>
  <circle cx="167" cy="50" r="5" fill="#94a3b8"/>
  <circle cx="239" cy="50" r="5" fill="#94a3b8"/>
  <circle cx="311" cy="50" r="5" fill="#94a3b8"/>
  <circle cx="383" cy="50" r="5" fill="#94a3b8"/>
  <circle cx="455" cy="50" r="5" fill="#94a3b8"/>
  <circle cx="527" cy="50" r="5" fill="#94a3b8"/>
  <text x="88" y="36" fill="#9ca3af" font-size="11" font-family="monospace">t₀</text>
  <text x="520" y="36" fill="#9ca3af" font-size="11" font-family="monospace">t_f</text>

  <!-- ROW 2: MPC 1 -->
  <line x1="95" y1="100" x2="530" y2="100" stroke="#6b7280" stroke-width="1.5"/>
  <circle cx="95" cy="100" r="5" fill="#f87171"/>
  <circle cx="119" cy="100" r="5" fill="#f87171"/>
  <circle cx="167" cy="100" r="5" fill="#94a3b8"/>
  <circle cx="239" cy="100" r="5" fill="#94a3b8"/>
  <circle cx="311" cy="100" r="5" fill="#94a3b8"/>
  <circle cx="383" cy="100" r="5" fill="#94a3b8"/>
  <circle cx="455" cy="100" r="5" fill="#94a3b8"/>
  <circle cx="527" cy="100" r="5" fill="#94a3b8"/>
  <text x="88" y="120" fill="#f87171" font-size="10" font-family="monospace">t₀</text>
  <text x="113" y="120" fill="#f87171" font-size="10" font-family="monospace">t₁</text>
  <text x="520" y="88" fill="#9ca3af" font-size="10" font-family="monospace">t_f</text>

  <!-- ROW 3: MPC 2 -->
  <line x1="143" y1="150" x2="530" y2="150" stroke="#6b7280" stroke-width="1.5"/>
  <circle cx="143" cy="150" r="5" fill="#f87171"/>
  <circle cx="167" cy="150" r="5" fill="#f87171"/>
  <circle cx="239" cy="150" r="5" fill="#94a3b8"/>
  <circle cx="311" cy="150" r="5" fill="#94a3b8"/>
  <circle cx="383" cy="150" r="5" fill="#94a3b8"/>
  <circle cx="455" cy="150" r="5" fill="#94a3b8"/>
  <circle cx="527" cy="150" r="5" fill="#94a3b8"/>
  <text x="136" y="170" fill="#f87171" font-size="10" font-family="monospace">t₀</text>
  <text x="161" y="170" fill="#f87171" font-size="10" font-family="monospace">t₁</text>
  <text x="520" y="138" fill="#9ca3af" font-size="10" font-family="monospace">t_f</text>

  <!-- ROW 4: MPC k -->
  <line x1="407" y1="200" x2="530" y2="200" stroke="#6b7280" stroke-width="1.5"/>
  <circle cx="407" cy="200" r="5" fill="#f87171"/>
  <circle cx="431" cy="200" r="5" fill="#f87171"/>
  <circle cx="455" cy="200" r="5" fill="#94a3b8"/>
  <circle cx="527" cy="200" r="5" fill="#94a3b8"/>
  <text x="400" y="220" fill="#f87171" font-size="10" font-family="monospace">t₀</text>
  <text x="425" y="220" fill="#f87171" font-size="10" font-family="monospace">t₁</text>
  <text x="520" y="188" fill="#9ca3af" font-size="10" font-family="monospace">t_f</text>

  <!-- Vertical guide lines showing fixed points -->
  <line x1="167" y1="42" x2="167" y2="208" stroke="#374151" stroke-width="0.8" stroke-dasharray="3,4"/>
  <line x1="239" y1="42" x2="239" y2="208" stroke="#374151" stroke-width="0.8" stroke-dasharray="3,4"/>
  <line x1="311" y1="42" x2="311" y2="208" stroke="#374151" stroke-width="0.8" stroke-dasharray="3,4"/>
  <line x1="383" y1="42" x2="383" y2="208" stroke="#374151" stroke-width="0.8" stroke-dasharray="3,4"/>
  <line x1="455" y1="42" x2="455" y2="208" stroke="#374151" stroke-width="0.8" stroke-dasharray="3,4"/>

  <!-- Legend -->
  <line x1="14" y1="232" x2="566" y2="232" stroke="#374151" stroke-width="1"/>
  <circle cx="24" cy="248" r="5" fill="#f87171"/>
  <text x="36" y="252" fill="#f87171" font-size="10" font-family="monospace">t₀,t₁: her iterasyonda İLERİ KAYAN, yeni eklenen yoğun noktalar</text>
  <circle cx="360" cy="248" r="5" fill="#94a3b8"/>
  <text x="372" y="252" fill="#9ca3af" font-size="10" font-family="monospace">Referanstan gelen SABİT seyrek noktalar</text>
</svg>

**Diyagramdan okunanlar:**
- Referans yörüngenin seyrek noktaları (gri) **konumlarını hiç değiştirmiyor** — dikey kesikli çizgiler bunu gösteriyor
- $t_0,t_1$ (kırmızı) her iterasyonda **bir adım ileri kayıyor**
- Zaman ilerledikçe geride kalan noktalar ufuktan düşüyor → **optimizasyon problemi küçülüyor** (MPC k satırı: sadece 4 nokta kaldı)

**(2) Amaç fonksiyonu değişikliği:** Takip hatası yerine **doğrudan Problem2'nin $J=-z_f$'si** kullanılıyor:

> *"the new MPC problem is in line with the Problem2 except the additional discrete point."*

**Felsefi fark:**

| | Klasik MPC (Eq.25) | Wang & Song Novel MPC |
|---|---|---|
| Amaç | Referansa geri dön | Yakıtı optimize et |
| Büyük sapmada | Zorla referansa döner (yakıt israfı olabilir) | Mevcut durumdan **yeniden** yakıt-optimal yol hesaplar |

## 5.3 Diskretizasyon — Trapezoidal Rule

**Neden diskretizasyon gerekli:** Optimizasyon çözücüleri sürekli fonksiyonlarla çalışamaz; sürekli $\dot{x}=f(x,u)$'yu sonlu ayrık noktaya indirger, ama noktalar **fiziksel olarak tutarlı** kalmalı (rastgele "sıçrama" olmamalı).

**Forward Euler vs Trapezoidal — Açık Karşılaştırma:**

| | Forward Euler | Trapezoidal (Eq.26) |
|---|---|---|
| **Formül** | $x_i=x_{i-1}+\Delta t\cdot\dot{x}_{i-1}$ | $x_i=x_{i-1}+\frac{\Delta t}{2}(\dot{x}_{i-1}+\dot{x}_i)$ |
| **Kullandığı eğim** | Sadece başlangıç noktası | Başlangıç + bitiş **ortalaması** |
| **Yerel hata mertebesi** | $O(\Delta t^2)$ | $O(\Delta t^3)$ |
| **$\Delta t=1s$'de** | Kaba, birikimli hata büyük | Çok daha hassas |

Makale **trapezoidal seçiyor** çünkü 21 seyrek nokta ile büyük $\Delta t=1s$ aralık zorunlu — Euler'in hatası bu durumda kabul edilemez.

$$x_i=x_{i-1}+\frac{\Delta t_i}{2}(\dot{x}_{i-1}+\dot{x}_i),\quad i=1,\ldots,N\tag{26}$$

Bu, **her ardışık nokta çifti için** genel bir fiziksel tutarlılık kısıtı — tüm $N+1$ nokta aynı anda, tek bir optimizasyon probleminde birlikte çözülüyor.

**Lineer interpolasyonun kısıt ihlali — sayısal kanıt:**

Referans noktalar $T(0)=800$kN, $T(1)=1300$kN olsun. Lineer interpolasyon:
$$T(0.3s)\approx800+0.3\times(1300-800)=950\text{ kN}$$
Bu **hesaplanmış bir değer değil, düz çizgi varsayımı.** Gerçek optimal $T(0.3s)$ belki $1400$kN gerektirebilir (kısıtı aşar!) — ama interpolasyon bunu göremez. MPC optimizasyonunun $T_{min}\leq T\leq T_{max}$ kısıtını her noktada **içermesi**, bu ihlali **garantili önlüyor.**

### Optimizasyon Değişken Vektörü (Eq.27)

$$Variable_{opt}=\left[x_0^T,\ldots,x_N^T,\;u_0^T,\ldots,u_N^T\right]^T\tag{27}$$

Çözücüye gönderilen **tek bir uzun vektör** — tüm ayrık noktalardaki durum ve kontrol değerlerini içeriyor:

```
Variable_opt = [ x(t₀), x(t₁), x(nokta₂), ..., x(t_f),      ← DURUMLAR
                 u(t₀), u(t₁), u(nokta₂), ..., u(t_f) ]     ← KONTROLLER

Boyut = (N+1)·(durum sayısı) + (N+1)·(kontrol sayısı)
```

**§4.1'deki somut örnek:** 21 nokta, $x\in\mathbb{R}^6$, $u_1,u_2,u_3$ her noktada:

$$21\times6=126\text{ durum değişkeni},\qquad 21\times3=63\text{ kontrol değişkeni}$$

> **$t_f$ neden çarpılmıyor?** $u=[u_1,u_2,u_3,t_f]^T$ olsa da, $t_f$ **tüm noktalarda ortak tek bir skaler** — her nokta için ayrı $t_f$ yok. Bu yüzden $21\times4=84$ değil, $21\times3=63$ (+1 tek $t_f$).

**Her şey bu tek vektör üzerinden tanımlanıyor:**

| Ne | Vektörün neresinde |
|---|---|
| Sınır koşulları (Eq.3) | $x_0$ ve $x_N$ elemanları sabitlenir |
| Süreç kısıtları (Eq.14) | Her $u_i$ için ayrı ayrı |
| SOC kısıtı (Eq.22) | Her $u_i$ için ayrı ayrı |
| Dinamik kısıt (Eq.26) | Ardışık $x_{i-1},x_i$ çiftleri arasında |
| Amaç fonksiyonu (Eq.12) | Sadece $z_N$ elemanına bağlı |

Bu yapı sayesinde $t_0$, $t_1$ ve tüm diğer noktalar **tek bir optimizasyon çağrısında, birbirleriyle tutarlı** olarak çözülüyor — ayrı ayrı problemler değil.

## 5.4 Hesaplama Verimliliği — Warm Start

Bir önceki MPC iterasyonunun çözümü, sonrakinin başlangıç tahmini olarak kullanılıyor — ardışık adımlar arası durum farkı küçük olduğundan, successive linearization çok daha az iterasyonla yakınsıyor.

---

# 6. Doğrulama — Section 4: Sayısal Sonuçlar

## 6.1 §4.1 — Yörünge Optimizasyonu

**Kurulum:** MOSEK çözücü, i7-4790/4GB RAM, 21 ayrık nokta → 126 durum + 63 kontrol değişkeni.

**Roket parametreleri (Tablo 1):** $m_0=55000$kg, $m_{dry}=49000$kg (6000kg kullanılabilir yakıt), $I_{sp}=443$s ($v_e\approx4346$m/s), $[T_{min},T_{max}]=[412.7,1375.6]$kN (throttle oranı ~3.33).

**Koşullar (Tablo 2):** $h_0=3$km, $s_f=1$km, $V_0=280$m/s, $\gamma_0=-65°$.

**Ana sonuç — serbest $t_f$'nin avantajı:**

| Senaryo | Uçuş Süresi | Yakıt |
|---|---|---|
| $t_f$ sabit (20s) | 20.0s | 5489 kg |
| $t_f$ serbest | **19.28s** | **5410 kg** (−79kg, %1.44 tasarruf) |
| Runge-Kutta doğrulama | 19.3s | — |

**Runge-Kutta doğrulamasındaki sapma:** Terminal $\gamma_f$'de küçük hata — nedeni, doğrulamanın **21 seyrek noktayı lineer interpolasyonla** doldurması (tam olarak Section 3'ün eleştirdiği sorun, burada somut örnekle görülüyor).

## 6.2 §4.2 — MPC Sonuçları

**Zamanlama:** Güdüm döngüsü 0.1s, yörünge aralığı 1s.

**Sapma senaryoları (Tablo 3):** $\Delta h=100$m, $\Delta s=100$m, $\Delta V=20$m/s, $\Delta\gamma=3°$ — hepsi **başlangıç anına** ($t_0$) uygulanan tek seferlik sapmalar; hepsi pozitif yönde yakıt tüketimini artırıyor.

| Senaryo | Yakıt | Süre | Not |
|---|---|---|---|
| Sapma yok | 5419 kg | — | Terminal $\gamma_f$ **tam** $-90°$'ye ulaşıyor |
| Negatif sapma | **5198 kg** | 19.2s | Daha az iş gerektiren başlangıç |
| Pozitif sapma | **5777 kg** | 19.6s | **Geriden yaklaşma manevrası** |

**"Geriden yaklaşma" manevrası:** Pozitif sapmada roket çok fazla enerjiye ama az yatay mesafeye sahip — hedefi **geçip, ters yönden geri dönerek** iniyor. $\gamma$, $-90°$'yi geçici olarak **aşıp**, terminal anda tam $-90°$'ye dönüyor. Bu, "global optimization objective" felsefesinin somut kanıtı — MPC referansa zorla uymuyor, mevcut enerjiden en iyi çözümü buluyor.

**MPC'nin düşük maliyetli yüksek faydası:**

$$\Delta m_{MPC}=5419-5410=9\text{ kg}\;(\%0.17)$$

Bu ihmal edilebilir ek maliyetle, §4.1'in çözemediği terminal açı hassasiyeti **tamamen çözülüyor.**

---

# 7. Sonuç ve 6-DOF Tez Entegrasyonu

## 7.1 Makalenin İki Ana Katkısı → Tezin İki Yapı Taşı

```
Katkı 1: Serbest tf (Eq.8-9)     → Tez: 6-DOF ÜST katman (global yörünge)
Katkı 2: Novel Receding Horizon  → Tez: 6-DOF ALT katman (anlık güdüm)
```

## 7.2 6-DOF Durum/Kontrol Genişlemesi

| 2-DOF | 6-DOF Karşılığı |
|---|---|
| $x=[r,s,V,\gamma,z,t]^T$ ($\in\mathbb{R}^6$) | $x=[\mathbf{p},\mathbf{v},\mathbf{q},\boldsymbol\omega,z,t]^T$ ($\in\mathbb{R}^{13+}$) |
| $\dot{V}$ (teğet Newton) | $m\dot{\mathbf{v}}=\mathbf{T}+\mathbf{D}+m\mathbf{g}$ (3 ayrı eksen) |
| $\dot\gamma$ (normal Newton) | $\mathbf{I}\dot{\boldsymbol\omega}+\boldsymbol\omega\times\mathbf{I}\boldsymbol\omega=\boldsymbol\tau$ (Euler rotasyon) |
| $\dot{m}=-T/I_{sp}$ | $\dot{m}=-\|\mathbf{T}\|/I_{sp}$ (doğrudan taşınır) |
| $u_1^2+u_2^2=u_3^2$ (2D SOC) | $u_1^2+u_2^2+u_3^2=u_4^2$ (3D SOC, aynı relaxation mantığı) |

## 7.3 Angara 1.2 Modeline Özel Kararlar

- **Sloshing yok**, ama CG yakıt tükendikçe kayıyor → $r_{CG}(t)$ deterministik hesaplanır, $\mathbf{I}(t)$ her adımda güncellenir
- **Quaternion normalizasyonu** her adımda uygulanacak ($q\leftarrow q/\|q\|$)
- **Ekstra nokta sayısı** ($t_1,t_2,\ldots$): sabit değil, test edilerek belirlenecek parametre
- **Başlangıç hedefi:** sadece düz dikey iniş ($\theta_f\approx0°$, $\phi_f\approx0°$); yapısal yük/açısal hız/gözlem alanı kısıtları sonradan eklenecek

## 7.4 Açık Sorular / İleri Çalışma

- **SOC relaxation'ın 3D'de kayıpsızlığı** — Liu [11] 2D için kanıtlıyor; 6-DOF için ayrıca kanıtlanmalı ya da Szmuk & Açıkmeşe gibi literatürden desteklenmeli
- **Hesaplama süresi ölçeklemesi:** 2-DOF'ta 189 karar değişkeni (126+63) idi; 6-DOF'ta tahmini ~378 (2 kat). SOCP karmaşıklığı yüksek üslü olduğundan (tipik SOCP literatüründe $O(n^3)$-$O(n^4)$ aralığı — bu makalede kesin üs verilmiyor, sadece "polynomial time" deniyor), **gerçek ölçüm** yapılmalı, teorik tahmine güvenilmemeli
- **Hiyerarşik mimaride $J=-z_f$'nin her iki katmanda da mı, yoksa alt katmanda farklı bir kriterle mi kullanılacağı** — tez metodolojisinde netleştirilmesi gereken tasarım kararı

---

# Ek: Referans Denklem İndeksi

| Eq. | İçerik | Bölüm |
|---|---|---|
| 1a-1e | Nokta-kütle dinamik denklemler | §2.1 |
| 2 | Sürükleme kuvveti | §2.1 |
| 3-5 | Sınır koşulları, terminal/süreç kısıtları | §2.1 |
| 6-7 | Amaç fonksiyonu, Problem0 | §2.2 |
| 8-9 | Zaman normalizasyonu, augmented dynamics | §2.3 |
| 10-11 | Değişken dönüşümü, Pisagor kısıtı | §2.3 |
| 12-16 | Yeni amaç/kısıtlar, Problem1 | §2.3 |
| 17-21 | Successive linearization, $e^{-z}$ doğrusallaştırma | §2.3 |
| 22-24 | SOC relaxation, Problem2, yakınsama kriteri | §2.3 |
| 25 | Klasik MPC amaç fonksiyonu (kullanılmayan, karşılaştırma) | Section 3 |
| 26-27 | Trapezoidal diskretizasyon, karar değişkeni vektörü | Section 3 |

