---
tags:
  - makale-analizi
  - wang-song-2018
  - bolum-2-1
  - dinamik-denklemler
  - nokta-kutle
  - kinetik
  - kisitlar
  - roket-inisi
  - MPC
kaynak: "Wang, C. & Song, Z. (2018). Convex Model Predictive Control for Rocket Vertical Landing. *Proceedings of the 37th Chinese Control Conference*, Wuhan, pp. 9837–9842."
bolum: "§2.1 — Dynamics and Constraints"
durum: adim-4-tamamlandi
ilgili: "[[Angara_1.2_6DOF_Simulink_Modeli]]"
---

# §2.1 — Dinamik Denklemler ve Kısıtlar

> **Kaynak:** Wang & Song (2018), Section 2, §2.1
> **Appendixler:** [Kinematik](Appendixes/wang_song_2018_kinematik.html) · [Alfa Geometrisi](Appendixes/wang_song_2018_alpha_geometrisi.html) · [Gamma Kuvvet Dengesi](Appendixes/wang_song_2018_gammadot.html)

---

## Temel Varsayımlar

- **İtki yönü = Roket ekseni:** Motor itkisi her zaman gövde eksenine paralel.
- **Silindirik gövde:** $C_L = 0$; yalnızca sürükleme modellenir.
- **Uzunlamasına düzlem (2-DOF):** Dikey $r$ ve yatay $s$ eksen; yan dinamikler yok.
- **Boyutsuzlaştırılmış değişkenler:** Ölçekler Ref. 11 Liu (2017)'dan alınmıştır.

---

## Durum ve Kontrol Değişkenleri

| Sembol   | Tanım                    | Not                                                    |
| -------- | ------------------------ | ------------------------------------------------------ |
| $r$      | Radyal konum (yükseklik) | Yer merkezinden uzaklık                                |
| $s$      | Yatay konum (menzil)     |                                                        |
| $V$      | Hız büyüklüğü (skaler)   | Yönü $\gamma$ ile belirlenir                           |
| $\gamma$ | Uçuş yolu açısı          | Yerel yatay eksenle; iniş → negatif                    |
| $m$      | Anlık kütle              | Yakıt tükendikçe azalır                                |
| $T$      | İtki büyüklüğü           | Kontrol; $T_{\min} \leq T \leq T_{\max}$               |
| $\alpha$ | Hücum açısı              | Kontrol; $-\hat{e}_t$ (anti-velocity) ekseninden sapma |

---

## Nokta-Kütle Uçuş Dinamiği — Koordinat Sistemi ve Türetim

### İçsel Koordinat Sistemi (Intrinsic Coordinates)

Nokta-kütle modeli, sabit bir dünya ekseni yerine **hareketle birlikte dönen içsel eksenler** kullanır:

$$\hat{e}_t = \begin{pmatrix}\sin\gamma \\ \cos\gamma\end{pmatrix}_{(r,s)} \quad \text{(teğet: hız yönü)}$$

$$\hat{e}_n = \frac{d\hat{e}_t}{d\gamma} = \begin{pmatrix}\cos\gamma \\ -\sin\gamma\end{pmatrix}_{(r,s)} \quad \text{(normal: hıza dik)}$$

**Geometrik doğrulama:** $\hat{e}_t \cdot \hat{e}_n = \sin\gamma\cos\gamma + \cos\gamma(-\sin\gamma) = 0$ ✓

### Türetim 1 — $\dot{r}$ ve $\dot{s}$: Kinematik

Hız vektörü içsel koordinatlarda:

$$\vec{V} = V\hat{e}_t = V\begin{pmatrix}\sin\gamma \\ \cos\gamma\end{pmatrix}$$

$(r,s)$ eksenlerine doğrudan yansıtılır:

$$\boxed{\dot{r} = V\sin\gamma, \quad \dot{s} = V\cos\gamma}$$

### Türetim 2 — $\dot{V}$: Teğet Eksen ($\hat{e}_t$) Newton 2. Yasası

---

**Adım 1 — Hız vektörünü içsel koordinatlarda tanımla:**

Roketin anlık hızı, büyüklük $V$ (skaler hız) ve yön $\hat{e}_t$ (birim teğet vektör) cinsinden:

$$\vec{V} = V\hat{e}_t$$

Bu noktada $V$ ve $\hat{e}_t$'nin **ikisi de zamana bağlı** değişkendir. Zaman türevi alınırken **çarpım kuralı** uygulanmalıdır.

---

**Adım 2 — Zamanla türevini al (çarpım kuralı):**

$$\frac{d\vec{V}}{dt} = \frac{d}{dt}\bigl(V\hat{e}_t\bigr) = \underbrace{\dot{V}\hat{e}_t}_{\substack{\text{büyüklük değişimi} \\ V\text{ artıyor/azalıyor}}} + \underbrace{V\dot{\hat{e}}_t}_{\substack{\text{yön değişimi} \\ \hat{e}_t\text{ dönüyor}}}$$

İlk terim açık: skaler hız değişimi. İkinci terim bilinmiyor — $\dot{\hat{e}}_t$ nedir?

---

**Adım 3 — $\dot{\hat{e}}_t = \dot{\gamma}\hat{e}_n$ olduğunu türet (en kritik adım):**

$\hat{e}_t$'yi $\gamma$ cinsinden açık olarak yaz:

$$\hat{e}_t(\gamma) = \begin{pmatrix}\sin\gamma \\ \cos\gamma\end{pmatrix}_{(r,s)}$$

Zamanla türev almak için **zincir kuralı** uygula ($\hat{e}_t$, $\gamma$'ya bağlı, $\gamma$ ise zamana bağlı):

$$\dot{\hat{e}}_t = \frac{d\hat{e}_t}{d\gamma}\cdot\frac{d\gamma}{dt} = \frac{d\hat{e}_t}{d\gamma}\cdot\dot{\gamma}$$

$\frac{d\hat{e}_t}{d\gamma}$'yi hesapla (bileşen bileşen türev al):

$$\frac{d\hat{e}_t}{d\gamma} = \frac{d}{d\gamma}\begin{pmatrix}\sin\gamma \\ \cos\gamma\end{pmatrix} = \begin{pmatrix}\cos\gamma \\ -\sin\gamma\end{pmatrix}$$

Ama bu tam olarak $\hat{e}_n$'nin tanımı! $\hat{e}_n \stackrel{\text{def}}{=} \frac{d\hat{e}_t}{d\gamma} = \begin{pmatrix}\cos\gamma \\ -\sin\gamma\end{pmatrix}$

Dolayısıyla:

$$\boxed{\dot{\hat{e}}_t = \dot{\gamma}\,\hat{e}_n}$$

> **Geometrik anlam:** $\hat{e}_t$ ucu birim çember üzerinde hareket eder. Çemberdeki herhangi bir noktada, harekete teğet yön o noktanın 90° döndürülmüşü — yani $\hat{e}_n$'dir. $\gamma$ hızla değişiyorsa ($\dot{\gamma}$ büyük) $\hat{e}_t$ hızla dönüyor; yavaş değişiyorsa ($\dot{\gamma}$ küçük) yavaş dönüyor. Bu yüzden $\dot{\hat{e}}_t$'nin büyüklüğü $|\dot{\gamma}|$, yönü ise daima $\hat{e}_n$'dir.

---

**Adım 4 — Türevleri birleştir:**

Adım 2 ve 3'ü birleştirince ivme vektörü:

$$\frac{d\vec{V}}{dt} = \dot{V}\hat{e}_t + V\dot{\gamma}\hat{e}_n$$

Newton 2. Yasa:

$$m\frac{d\vec{V}}{dt} = \vec{F}_{toplam} \quad\Longrightarrow\quad m\bigl(\dot{V}\hat{e}_t + V\dot{\gamma}\hat{e}_n\bigr) = \vec{F}_{toplam}$$

Bu tek denklem, **iki bileşeni** barındırıyor. Her eksene ayrı ayrı yansıtarak iki skaler denklem elde edeceğiz.

---

**Adım 5 — $\hat{e}_t$ eksenine yansıt** (her iki tarafı $\hat{e}_t$ ile iç çarp):

$$m\Bigl(\dot{V}\underbrace{(\hat{e}_t\cdot\hat{e}_t)}_{=1} + V\dot{\gamma}\underbrace{(\hat{e}_t\cdot\hat{e}_n)}_{=0}\Bigr) = \vec{F}_{toplam}\cdot\hat{e}_t$$

$$m\dot{V} = \vec{F}_{toplam}\cdot\hat{e}_t$$

$\hat{e}_t\cdot\hat{e}_n = 0$ çünkü $\hat{e}_t \perp \hat{e}_n$. Bu yüzden $V\dot{\gamma}$ terimi yok oluyor ve yalnızca $\dot{V}$ kalıyor.

---

**Adım 6 — Her kuvvetin $\hat{e}_t$ bileşenini hesapla:**

**Yerçekimi:** Yerçekimi $(r,s)$ uzayında saf $-r$ yönünde, yani $\vec{g} = (-g,\,0)^T$:

$$\vec{g}\cdot\hat{e}_t = \begin{pmatrix}-g \\ 0\end{pmatrix}\cdot\begin{pmatrix}\sin\gamma \\ \cos\gamma\end{pmatrix} = -g\sin\gamma$$

Boyutsuzlaştırılmış formda $g \to 1/r^2$:

$$\vec{g}\cdot\hat{e}_t = -\frac{\sin\gamma}{r^2}$$

*İniş sırasında* $\gamma < 0 \Rightarrow \sin\gamma < 0 \Rightarrow -\sin\gamma/r^2 > 0$ → yerçekimi $\hat{e}_t$ doğrultusunda bileşen üretiyor → **hızı artırıyor** ✓ (roket düşerken hızlanır)

**İtki:** $\vec{T}$, $-\hat{e}_t$ ekseninden $\alpha$ kadar $-\hat{e}_n$ yönüne saptırılmıştır. $\hat{e}_t$ bileşeni:

$$\vec{T}\cdot\hat{e}_t = -T\cos\alpha$$

$\alpha = 0$ olsaydı itki tam $-\hat{e}_t$ yönünde olurdu → $\hat{e}_t$ bileşeni $= -T$ (saf frenleme). $\alpha > 0$ için $\cos\alpha < 1$ → frenleme gücü azalır.

**Sürükleme:** $\vec{D}$ daima harekete karşı, yani $-\hat{e}_t$ yönünde:

$$\vec{D}\cdot\hat{e}_t = -D$$

---

**Sonuç — Teğet kuvvet dengesi:**

$$m\dot{V} = -T\cos\alpha - D - \frac{m\sin\gamma}{r^2}$$

$$\boxed{\dot{V} = \frac{-T\cos\alpha - D}{m} - \frac{\sin\gamma}{r^2}} \tag{1c}$$

---

### Türetim 3 — $\dot{\gamma}$: Normal Eksen ($\hat{e}_n$) Newton 2. Yasası

Adım 4'teki Newton denklemini bu sefer **$\hat{e}_n$ eksenine yansıt** (her iki tarafı $\hat{e}_n$ ile iç çarp):

$$m\Bigl(\dot{V}\underbrace{(\hat{e}_n\cdot\hat{e}_t)}_{=0} + V\dot{\gamma}\underbrace{(\hat{e}_n\cdot\hat{e}_n)}_{=1}\Bigr) = \vec{F}_{toplam}\cdot\hat{e}_n$$

$$mV\dot{\gamma} = \vec{F}_{toplam}\cdot\hat{e}_n$$

> **Neden sol tarafta $mV\dot{\gamma}$?** Aynı ivme ifadesinin $\hat{e}_n$ bileşeni: $V\dot{\gamma}$ terimi geliyor. $V$ paydadaki değil, **çarpandadır**. Sol tarafı $mV$'ye böleceğiz, bu yüzden $V$ paydaya geçecek.
>
> **Fiziksel anlam:** Aynı yön değiştirici kuvvet, **yüksek hızda daha az $\dot{\gamma}$** üretir — yüksek hızlı bir araç yönünü değiştirmek için daha büyük kuvvete ihtiyaç duyar.

---

**Her kuvvetin $\hat{e}_n$ bileşenini hesapla:**

**Yerçekimi:** $\vec{g} = (-g,\,0)^T$, $\hat{e}_n = (\cos\gamma,\,-\sin\gamma)^T$:

$$\vec{g}\cdot\hat{e}_n = \begin{pmatrix}-g \\ 0\end{pmatrix}\cdot\begin{pmatrix}\cos\gamma \\ -\sin\gamma\end{pmatrix} = -g\cos\gamma$$

Boyutsuzlaştırılmış: $-\cos\gamma/r^2$

*İniş sırasında* $\gamma < 0$ ama $|\gamma| < 90° \Rightarrow \cos\gamma > 0 \Rightarrow -\cos\gamma/r^2 < 0$ → yerçekimi $-\hat{e}_n$ yönünde bileşen üretiyor → $\dot{\gamma} < 0$ katkısı → yörünge daha da dikleşiyor ✓

**İtki:** $\hat{e}_n$ bileşeni:

$$\vec{T}\cdot\hat{e}_n = -T\sin\alpha$$

$\alpha > 0 \Rightarrow \sin\alpha > 0 \Rightarrow -T\sin\alpha < 0$ → $-\hat{e}_n$ yönünde bileşen → $\dot{\gamma} < 0$ katkısı → yörünge $-90°$'ye kıvrılıyor ✓

**Sürükleme:** $\vec{D}$ tamamen $-\hat{e}_t$ yönünde; $\hat{e}_t \perp \hat{e}_n$ olduğundan:

$$\vec{D}\cdot\hat{e}_n = 0$$

---

**Sonuç — Normal kuvvet dengesi:**

$$mV\dot{\gamma} = -T\sin\alpha - \frac{m\cos\gamma}{r^2}$$

Her iki tarafı $mV$'ye böl:

$$\boxed{\dot{\gamma} = \frac{-T\sin\alpha}{mV} - \frac{\cos\gamma}{r^2V}} \tag{1d}$$

**Kritik zincir:**

$$\alpha > 0 \;\Rightarrow\; -T\sin\alpha < 0 \;\Rightarrow\; \dot{\gamma} < 0 \;\Rightarrow\; \gamma \searrow -90° \checkmark$$

### Türetim 4 — $\dot{m}$: Tsiolkovsky / $I_{sp}$

İtki kuvveti = atılan gazın momentum değişimi:

$$T = v_e\cdot(-\dot{m}) \quad \Longrightarrow \quad \dot{m} = -\frac{T}{v_e}$$

$I_{sp}$ tanımı: $v_e = I_{sp}\cdot g_0 \Rightarrow \dot{m} = -T/(I_{sp}\cdot g_0)$. Boyutsuz formda $g_0=1$:

$$\boxed{\dot{m} = -\frac{T}{I_{sp}}}$$

---

## Denklem (1): Boyutsuz Dinamik Denklemler

### 1a–1b · $\dot{r}$ ve $\dot{s}$ — Kinematik

<svg width="500" height="290" viewBox="0 0 500 290" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- Local horizon -->
  <line x1="10" y1="150" x2="488" y2="150" stroke="#374151" stroke-width="1.5"/>
  <text x="330" y="143" fill="#6b7280" font-size="11" font-family="monospace">local ufuk (s ekseni)</text>
  <!-- r axis dashed -->
  <line x1="115" y1="12" x2="115" y2="282" stroke="#374151" stroke-width="1" stroke-dasharray="4,3"/>
  <text x="118" y="24" fill="#6b7280" font-size="11" font-family="monospace">r (dikey, +yukari)</text>
  <!-- Origin -->
  <circle cx="115" cy="150" r="4" fill="#e2e8f0"/>
  <text x="98" y="167" fill="#6b7280" font-size="11" font-family="monospace">O</text>
  <!-- γ arc: 65° CW from rightward, radius=55 -->
  <!-- start:(170,150), end:(115+55*0.423,150+55*0.906)=(138.3,199.8) -->
  <path d="M 170 150 A 55 55 0 0 1 138 200" stroke="#c084fc" stroke-width="2" fill="none"/>
  <!-- γ label at 32.5°: (115+68*0.843,150+68*0.537)=(172,187) -->
  <text x="175" y="187" fill="#c084fc" font-size="16" font-family="monospace" font-style="italic">γ</text>
  <text x="176" y="202" fill="#c084fc" font-size="10" font-family="monospace">(−65°)</text>
  <!-- s_dot: (115,150)→(159,150), green horizontal -->
  <line x1="115" y1="150" x2="153" y2="150" stroke="#4ade80" stroke-width="3"/>
  <polygon points="159,150 149,146 149,154" fill="#4ade80"/>
  <!-- r_dot: (159,150)→(159,245), blue vertical downward -->
  <line x1="159" y1="150" x2="159" y2="238" stroke="#60a5fa" stroke-width="3"/>
  <polygon points="159,244 155,234 163,234" fill="#60a5fa"/>
  <!-- Right-angle marker -->
  <polyline points="151,150 151,158 159,158" stroke="#6b7280" stroke-width="1.2" fill="none"/>
  <!-- V vector: (115,150)→(159,244) -->
  <line x1="115" y1="150" x2="159" y2="240" stroke="#f59e0b" stroke-width="3"/>
  <polygon points="159,244 151,237 158,229" fill="#f59e0b"/>
  <text x="165" y="243" fill="#f59e0b" font-size="15" font-weight="bold" font-family="monospace">V</text>
  <!-- s_dot label: below horizon, clear space -->
  <text x="118" y="172" fill="#4ade80" font-size="13" font-weight="bold" font-family="monospace">s&#775; = V cos&#947;</text>
  <text x="118" y="186" fill="#4ade80" font-size="11" font-family="monospace">= +0.423 · V  (saga)</text>
  <!-- r_dot label: right of blue arrow, clear space -->
  <text x="167" y="200" fill="#60a5fa" font-size="13" font-weight="bold" font-family="monospace">r&#775; = V sin&#947;</text>
  <text x="167" y="214" fill="#60a5fa" font-size="11" font-family="monospace">= −0.906 · V  (asagi)</text>
  <!-- Info panel -->
  <rect x="290" y="48" width="200" height="140" rx="6" fill="#111827" stroke="#374151" stroke-width="1"/>
  <text x="300" y="68" fill="#a78bfa" font-size="11" font-weight="bold" font-family="monospace">γ = −65° icin:</text>
  <line x1="300" y1="74" x2="480" y2="74" stroke="#374151" stroke-width="1"/>
  <text x="300" y="91" fill="#4ade80" font-size="12" font-family="monospace">s&#775; = +0.423 · V</text>
  <text x="300" y="108" fill="#60a5fa" font-size="12" font-family="monospace">r&#775; = −0.906 · V</text>
  <line x1="300" y1="116" x2="480" y2="116" stroke="#374151" stroke-width="1"/>
  <text x="300" y="132" fill="#f59e0b" font-size="11" font-family="monospace">|V|&#178; = s&#775;&#178; + r&#775;&#178;  (Pisagor)</text>
  <line x1="300" y1="140" x2="480" y2="140" stroke="#374151" stroke-width="1"/>
  <text x="300" y="156" fill="#6b7280" font-size="10" font-family="monospace">γ = 0°  → r&#775;=0, s&#775;=V</text>
  <text x="300" y="170" fill="#6b7280" font-size="10" font-family="monospace">γ =−90° → r&#775;=−V, s&#775;=0</text>
  <text x="300" y="184" fill="#6b7280" font-size="10" font-family="monospace">(terminal hedef)</text>
</svg>

$$\dot{r} = V\sin\gamma \tag{1a}$$
$$\dot{s} = V\cos\gamma \tag{1b}$$

---

### 1c · $\dot{V}$ — Teğet Eksen ($\hat{e}_t$) Kuvvet Dengesi

> Newton 2. Yasa $\hat{e}_t$ eksenine yansıtılmıştır: $m\dot{V} = \vec{F}\cdot\hat{e}_t$

<svg width="520" height="310" viewBox="0 0 520 310" xmlns="http://www.w3.org/2000/svg" style="background:#1a1b26;border-radius:8px;display:block;margin:10px 0">
  <!-- Local horizon at y=155 -->
  <line x1="10" y1="155" x2="360" y2="155" stroke="#374151" stroke-width="1.5"/>
  <text x="240" y="148" fill="#6b7280" font-size="11" font-family="monospace">local ufuk</text>
  <!-- Origin at (160,155) on horizon -->
  <circle cx="160" cy="155" r="4" fill="#e2e8f0"/>
  <!-- ê_t axis: 45° lower-right (schematic) -->
  <line x1="160" y1="155" x2="242" y2="237" stroke="#f97316" stroke-width="1.5" stroke-dasharray="6,3"/>
  <polygon points="245,240 234,235 239,225" fill="#f97316"/>
  <text x="248" y="244" fill="#f97316" font-size="13" font-weight="bold" font-family="monospace">e&#770;&#8348;</text>
  <!-- ê_n axis: 45° upper-right -->
  <line x1="160" y1="155" x2="242" y2="73" stroke="#a78bfa" stroke-width="1.5" stroke-dasharray="6,3"/>
  <polygon points="245,70 234,76 239,86" fill="#a78bfa"/>
  <text x="248" y="72" fill="#a78bfa" font-size="13" font-weight="bold" font-family="monospace">e&#770;&#8345;</text>
  <!-- anti-ê_t dashed reference (for α arc) -->
  <line x1="160" y1="155" x2="78" y2="73" stroke="#f87171" stroke-width="1" stroke-dasharray="4,3" opacity="0.5"/>
  <text x="48" y="70" fill="#f87171" font-size="10" font-family="monospace" opacity="0.7">−e&#770;&#8348;</text>
  <!-- Thrust T (at α=20° from anti-ê_t toward -ê_n) -->
  <!-- T_screen = -cos20°*(0.707,0.707) - sin20°*(0.707,-0.707) = (-0.907,-0.423) -->
  <!-- T endpoint at sc=82: (160-74.4,155-34.7)=(85.6,120.3)→(86,120) -->
  <line x1="160" y1="155" x2="90" y2="123" stroke="#f87171" stroke-width="3"/>
  <polygon points="87,121 98,124 95,134" fill="#f87171"/>
  <text x="42" y="118" fill="#f87171" font-size="13" font-weight="bold" font-family="monospace">T (itki)</text>
  <!-- α arc: from anti-ê_t dir to T dir, radius=32 -->
  <!-- anti-ê_t at r=32: (160+32*(-0.707),155+32*(-0.707))=(137.4,132.4)→(137,132) -->
  <!-- T at r=32: (160+32*(-0.907),155+32*(-0.423))=(131,141.5)→(131,142) -->
  <path d="M 137 132 A 32 32 0 0 1 131 142" stroke="#fbbf24" stroke-width="1.5" fill="none"/>
  <text x="115" y="128" fill="#fbbf24" font-size="12" font-family="monospace" font-style="italic">α</text>
  <!-- g vector: straight down from origin -->
  <line x1="160" y1="155" x2="160" y2="238" stroke="#818cf8" stroke-width="2.5"/>
  <polygon points="160,242 156,232 164,232" fill="#818cf8"/>
  <text x="164" y="240" fill="#818cf8" font-size="12" font-family="monospace">g</text>
  <!-- γ arc: from horizon to ê_t direction, radius=58 -->
  <!-- from (218,155) sweeping 45° CW to (160+58*0.707,155+58*0.707)=(201,196) -->
  <path d="M 218 155 A 58 58 0 0 1 201 196" stroke="#c084fc" stroke-width="1.5" fill="none"/>
  <!-- γ label at 22.5°: (160+68*0.924,155+68*0.383)=(223,181) -->
  <text x="222" y="182" fill="#c084fc" font-size="13" font-family="monospace" font-style="italic">γ</text>
  <!-- g component along ê_t (dashed blue along ê_t direction) -->
  <!-- -g sinγ for γ=-65°: positive → along +ê_t -->
  <!-- endpoint at sc=38: (160+27,155+27)=(187,182) -->
  <line x1="160" y1="155" x2="185" y2="180" stroke="#818cf8" stroke-width="1.5" stroke-dasharray="3,3"/>
  <polygon points="187,182 178,177 182,170" fill="#818cf8"/>
  <text x="188" y="180" fill="#818cf8" font-size="10" font-family="monospace">−g sinγ</text>
  <!-- T component along anti-ê_t (dashed red) -->
  <!-- -T cosα: along anti-ê_t direction, sc=45 -->
  <!-- endpoint: (160+45*(-0.707),155+45*(-0.707))=(128,123) -->
  <line x1="160" y1="155" x2="130" y2="125" stroke="#f87171" stroke-width="1.5" stroke-dasharray="3,3"/>
  <text x="98" y="138" fill="#f87171" font-size="10" font-family="monospace">−T cosα</text>
  <!-- Info panel (right) -->
  <rect x="338" y="28" width="176" height="255" rx="6" fill="#111827" stroke="#374151" stroke-width="1"/>
  <text x="348" y="48" fill="#a78bfa" font-size="11" font-weight="bold" font-family="monospace">e&#770;&#8348; bilesenleri:</text>
  <line x1="348" y1="54" x2="505" y2="54" stroke="#374151" stroke-width="1"/>
  <text x="348" y="71" fill="#f87171" font-size="11" font-family="monospace">−T cosα</text>
  <text x="348" y="84" fill="#6b7280" font-size="10" font-family="monospace">  itki fren bileseni</text>
  <text x="348" y="100" fill="#fb923c" font-size="11" font-family="monospace">−D</text>
  <text x="348" y="113" fill="#6b7280" font-size="10" font-family="monospace">  suruklenme</text>
  <text x="348" y="129" fill="#818cf8" font-size="11" font-family="monospace">−g sinγ / r²</text>
  <text x="348" y="142" fill="#6b7280" font-size="10" font-family="monospace">  yer cekimi tegeti</text>
  <text x="348" y="155" fill="#6b7280" font-size="10" font-family="monospace">  γ&lt;0 → pozitif (hizlanir)</text>
  <line x1="348" y1="163" x2="505" y2="163" stroke="#374151" stroke-width="1"/>
  <text x="348" y="180" fill="#e2e8f0" font-size="10" font-family="monospace">mV&#775; = toplam e&#770;&#8348; kuvvet</text>
  <line x1="348" y1="188" x2="505" y2="188" stroke="#374151" stroke-width="1"/>
  <text x="348" y="205" fill="#fbbf24" font-size="11" font-weight="bold" font-family="monospace">α acisi:</text>
  <text x="348" y="219" fill="#fbbf24" font-size="10" font-family="monospace">T, −e&#770;&#8348; ekseninden</text>
  <text x="348" y="232" fill="#fbbf24" font-size="10" font-family="monospace">α kadar saptirmis.</text>
  <text x="348" y="246" fill="#fbbf24" font-size="10" font-family="monospace">α=0 → saf frenleme</text>
  <text x="348" y="260" fill="#fbbf24" font-size="10" font-family="monospace">α&gt;0 → e&#770;&#8345; yonune sapma</text>
  <text x="348" y="276" fill="#6b7280" font-size="9" font-family="monospace">bkz. Alfa Geometrisi HTML</text>
</svg>

$$\boxed{\dot{V} = \frac{-T\cos\alpha - D}{m} - \frac{\sin\gamma}{r^2}} \tag{1c}$$

| Terim | Etki | İşaret ($\gamma < 0$, iniş) |
|-------|------|--------------------------|
| $-T\cos\alpha/m$ | İtki fren bileşeni | $< 0$ → $V$ azalır |
| $-D/m$ | Sürükleme (daima harekete karşı) | $< 0$ → $V$ azalır |
| $-\sin\gamma/r^2$ | Yerçekimi teğet bileşeni | $> 0$ → $V$ artar |

---

### 1d · $\dot{\gamma}$ — Normal Eksen ($\hat{e}_n$) Kuvvet Dengesi

> Newton 2. Yasa $\hat{e}_n$ eksenine yansıtılmıştır: $mV\dot{\gamma} = \vec{F}\cdot\hat{e}_n$
> Detaylı görsel: [→ Gamma Kuvvet Dengesi HTML](Appendixes/wang_song_2018_gammadot.html)

$$\boxed{\dot{\gamma} = \frac{-T\sin\alpha}{mV} - \frac{\cos\gamma}{r^2V}} \tag{1d}$$

| Terim | Etki | İşaret ($\alpha>0$, $\gamma<0$) |
|-------|------|--------------------------------|
| $-T\sin\alpha/(mV)$ | İtki normal bileşeni | $< 0$ → $\gamma$ azalır → $-90°$'ye |
| $-\cos\gamma/(r^2V)$ | Yerçekimi normal bileşeni | $< 0$ → $\gamma$ azalır |

> **Kritik zincir:** $\alpha > 0 \Rightarrow -T\sin\alpha < 0 \Rightarrow \dot{\gamma} < 0 \Rightarrow \gamma \to -90°$ ✓

---

### 1e · $\dot{m}$ — Kütle Tüketimi

$$\boxed{\dot{m} = -\frac{T}{I_{sp}}} \tag{1e}$$

$$I_{sp} = \frac{v_e}{g_0} \quad\Longleftrightarrow\quad v_e = I_{sp}\cdot g_0 \quad\Longleftrightarrow\quad T = v_e\cdot(-\dot{m})$$

- $v_e$: Egzoz gazı hızı (exhaust velocity) [m/s]
- $g_0 = 9.81$ m/s²: Standart yerçekimi ivmesi
- **Angara 1.2:** $I_{sp} = 443$ s $\Rightarrow$ $v_e \approx 4346$ m/s

---

## Denklem (2): Sürükleme Kuvveti

$$D = \frac{1}{2}\rho V^2 S_{ref} C_D, \qquad \rho = \rho_0 e^{-\beta h}, \qquad h = r - R_e \tag{2}$$

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| $S_{ref}$ | 8 m² | Referans alan |
| $C_D$ | 0.25 | Sürükleme katsayısı |
| $R_e$ | Dünya yarıçapı | Gerçek yükseklik: $h = r - R_e$ |

---

## Denklemler (3–5): Kısıtlar

### Sınır Koşulları — Denklem (3)

$$\mathbf{x}_0 = [3\,\text{km},\ 0,\ 280\,\text{m/s},\ {-65°},\ 55000\,\text{kg}]^T$$
$$\mathbf{x}_f = [0,\ 1\,\text{km},\ \leq V_{safe},\ -90°,\ \geq m_{dry}]^T$$

### Terminal Kısıtlar — Denklem (4)

$$V_f \leq 1\,\text{m/s}, \quad \gamma_f = -90°, \quad |\alpha_f| \leq 2°, \quad m_f \geq 49000\,\text{kg}$$

> ⚠️ **Onaylı Yazım Hatası:** Makale Eq.(4)'te $\gamma_f = 90°$ yazar. Fiziksel doğrusu $-90°$'dir (§4.2 ve Tablo 2 ile doğrulandı).

### Süreç Kısıtları — Denklem (5)

$$412.7\,\text{kN} \leq T \leq 1375.6\,\text{kN}, \qquad -10° \leq \alpha \leq +10°$$

> ⚠️ **Nonkonveksite:** $T_{\min} > 0$ → küme $\{0\} \cup [T_{\min}, T_{\max}]$ dışbükey değil. §2.3'te lossless relaxation ile çözülür.

---

## Appendix Linkleri

| Dosya                                                                   | İçerik                                                                |
| ----------------------------------------------------------------------- | --------------------------------------------------------------------- |
| [Kinematik HTML](Appendixes/wang_song_2018_kinematik.html)              | $\dot{r}$/$\dot{s}$ geometrisi, $\gamma$ kaydırıcılı                  |
| [Alfa Geometrisi HTML](Appendixes/wang_song_2018_alpha_geometrisi.html) | $\alpha$ açısı ve kuvvet bileşenleri                                  |
| [Gamma Kuvvet Dengesi HTML](Appendixes/wang_song_2018_gammadot.html)    | $\dot{\gamma}$ normal kuvvet dengesi, $\gamma$ ve $\alpha$ gösterimli |

---

## 6-DOF Entegrasyon Notları

| 2-DOF | 6-DOF Karşılığı | Not |
|-------|-----------------|-----|
| $\dot{r},\dot{s}$ | $\dot{\mathbf{p}} = \mathbf{v}$ | 3-eksen translasyon |
| $\dot{V}$ (teğet) | $m\dot{\mathbf{v}} = \mathbf{T} + \mathbf{D} + m\mathbf{g}$ | 3 ayrı denklem |
| $\dot{\gamma}$ (normal) | $\mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega}\times\mathbf{I}\boldsymbol{\omega} = \boldsymbol{\tau}$ | Euler rotasyon |
| $\dot{m}$ | $\dot{m} = -\|\mathbf{T}\|/I_{sp}$ | Doğrudan taşınır |

**Angara 1.2 modeline özel:**
- CG kayması deterministik: $r_{CG}(t)$ yakıt tüketiminden hesaplanır; $\mathbf{I}(t)$ her adımda güncellenir.
- Başlangıç hedefi: düz dikey iniş ($\theta_f \approx 0°$, $\phi_f \approx 0°$).
