---
tags:
  - makale-notu
  - powered-descent-guidance
  - sequential-convex-programming
  - state-triggered-constraints
  - continuous-time-constraint-satisfaction
  - 6-DOF
  - bitirme-tezi
kaynak: Uzun, S., Açıkmeşe, B., and Carson III, J. M., “Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-Time Compound State-Triggered Constraints,” AIAA SCITECH 2025 Forum, 2025, p. 1895. https://doi.org/10.2514/6.2025-1895
bolum: I. Introduction (tam)
durum: devam-ediyor
ilgili:
  - "[[Wang_Song_2018_KAPSAMLI_NOT]]"
  - "[[Angara_1.2_6DOF_Simulink_Modeli]]"
kod-deposu: https://github.com/sametuzun781/CT-cSTC
---

# Uzun, Açıkmeşe & Carson (2025) — Kapsamlı Not

> **Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-Time Compound State-Triggered Constraints**

---

## 0. Bu notun kapsamı ve nasıl okunmalı

Bu not, makalenin **Introduction bölümünün tam analizini** ve buna ek olarak **kod deposunun incelenmesini** içerir. Amaç, notu okuyan birinin makaleyi açmadan Introduction'da anlatılan literatür zincirini, kavramları ve bu kavramların tez mimarisine nasıl bağlandığını eksiksiz anlayabilmesidir.

Not **kümülatiftir** — sonraki bölümler (II. PDG Problemi, III. SCP Çözüm Metodu, IV. Sayısal Sonuçlar, V. Sonuç) analiz edildikçe bu dosyanın üzerine eklenecektir.

Yapı:

| Bölüm | İçerik                                              |
| ----- | --------------------------------------------------- |
| §1    | Künye ve kaynak/kod deposu durumu                   |
| §2    | Makalenin bir paragraflık özü                       |
| §3    | Introduction 1/2 — polinom yöntemlerden LCvx'e      |
| §4    | Introduction 2/2 — SCP'den D-GMSR'a                 |
| §5    | Derinleştirilmiş konular (kavramsal netleştirmeler) |
| §6    | Kod deposu analizi                                  |
| §7    | Tespitler, errata, makale–kod tutarsızlıkları       |
| §8    | Tez entegrasyonu ve uygulama yol haritası           |
| §9    | Açık sorular                                        |
| §10   | Terimler sözlüğü (tüm kısaltmaların açılımı)        |
| §11   | Referans haritası — hangi kaynak ne için okunmalı   |

---

## 1. Künye ve kaynak durumu

### 1.1 Tam künye (AIAA formatı)

> Uzun, S., Açıkmeşe, B., and Carson III, J. M., "Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-Time Compound State-Triggered Constraints," *AIAA SCITECH 2025 Forum*, 2025, p. 1895.
> https://doi.org/10.2514/6.2025-1895

BibTeX (kod deposunun README dosyasından alındı):

```bibtex
@inproceedings{uzun2025sequential,
  title={Sequential Convex Programming for 6-DoF Powered Descent Guidance
         with Continuous-Time Compound State-Triggered Constraints},
  author={Uzun, Samet and Acikmese, Behcet and Carson, John M},
  booktitle={AIAA SCITECH 2025 Forum},
  pages={1895},
  year={2025},
  url={https://doi.org/10.2514/6.2025-1895}
}
```

**Yazarlar ve kurumlar:**

| Yazar              | Kurum                             | Unvan                                                                     |
| ------------------ | --------------------------------- | ------------------------------------------------------------------------- |
| Samet Uzun         | University of Washington, Seattle | Doktora öğrencisi, William E. Boeing Dept. of Aeronautics & Astronautics  |
| Behçet Açıkmeşe    | University of Washington, Seattle | Profesör, AIAA Fellow                                                     |
| John M. Carson III | NASA Johnson Space Center         | Technical Integration Manager – Precision Landing, NASA STMD, AIAA Fellow |

**Fon kaynakları:** NASA Cooperative Agreement 80NSSC24M0212, AFOSR FA9550-20-1-0053, ONR N00014-20-1-2288.

### 1.2 Kod deposu — indirirken dikkat edilmesi gereken kritik nokta

Makalede verilen adres `https://github.com/UW-ACL/CT-cSTC` bir **sarmalayıcı (wrapper) depodur.** Asıl kod bu depoda değil, bir **git submodule** olarak bağlanmış ikinci bir depoda durur:

```
https://github.com/sametuzun781/CT-cSTC
```

> ⚠️ **GitHub'ın "Download ZIP" butonu submodule içeriğini indirmez.** İndirilen ZIP dosyasında `CT-cSTC/` klasörü **tamamen boş** çıkar. 18 MB'lık arşivin 17 MB'ı yalnızca bir animasyon GIF dosyasıdır; tek satır kod içermez.
>
> Doğru indirme yöntemi: `git clone --recursive https://github.com/UW-ACL/CT-cSTC.git`
> veya doğrudan `git clone https://github.com/sametuzun781/CT-cSTC.git`

Sarmalayıcı deponun içeriği:

| Dosya | Boyut | İçerik |
|---|---|---|
| `.gitmodules` | 89 B | Asıl deponun adresini veren bağlantı tanımı |
| `README.md` | 561 B | BibTeX künyesi + görsellere referans |
| `CT-cSTC/` | 0 | **Boş** — submodule içeriği burada olmalıydı |
| `rl_pos.png` | 884 KB | Makalenin Fig. 1'i (iniş yörüngesi) |
| `rl_oth.png` | 908 KB | Makalenin Fig. 2'si (durum/kontrol grafikleri) |
| `sim/rocket_landing.gif` | 17 MB | İniş animasyonu |

Asıl deponun içeriği ve ayrıntılı analizi §6'dadır.

---

## 2. Makalenin özü (bir paragrafta)

Makale iki ayrı tekniği birleştiriyor:

$$\underbrace{\text{D-GMSR}}_{\substack{\text{mantıksal kısıtları pürüzsüz,} \\ \text{sound ve complete şekilde} \\ \text{parametrize et}}} \;+\; \underbrace{\text{CT-SCvx}}_{\substack{\text{sürekli-zaman kısıt sağlama} \\ \text{garantisi ve yakınsama} \\ \text{garantisi ile çöz}}} \;=\; \text{Bu makale}$$

**Çözülen problem:** 6-DoF güdümlü motor inişi (PDG) probleminde, "eğer irtifa 100 m'nin altına inerse, o andan itibaren hız ≤ 20 m/s **ve** açısal hız ≤ 2.5 °/s **ve** tilt açısı ≤ 5° olsun" türünden **bileşik durum-tetiklemeli kısıtların (compound state-triggered constraints)**, yalnızca ayrıklaştırma düğümlerinde değil, **uçuş boyunca her anda** sağlandığını garanti etmek.

**Yöntemsel konum:** Makale LCvx (lossless convexification) kullanmaz — SCP (sequential convex programming) kullanır. Dolayısıyla garanti ettiği şey *global optimum* değil, *durağan noktaya yakınsama*dır. Bu ayrım §5.1'de ayrıntılı işlenmiştir.

---

## 3. Introduction — Bölüm 1/2: Polinom yöntemlerden LCvx'e

### 3.1 PDG nedir, neden önemli

**PDG (Powered Descent Guidance — Güdümlü Motor İnişi Rehberliği)**, bir roketin yörünge referansını ve ileri besleme (feedforward) kontrol komutlarını üreterek gezegen yüzeyine yumuşak inişini sağlayan problemdir.

Makale bunu insanlı ve insansız gezegen keşif görevlerinin *enabling technology*'si (mümkün kılıcı teknolojisi) olarak tanımlar: PDG çözülmeden Ay veya Mars'a hassas iniş yapılamaz.

### 3.2 Birinci nesil: Polinom rehberlik yöntemleri

Tarihsel olarak PDG, **polynomial guidance methods** (polinom rehberlik yöntemleri) ile çözülmüştür:

| Görev | Yıl | Ne indi |
|---|---|---|
| Apollo | 1969–1972 | Ay modülü, Ay yüzeyine |
| MSL (Mars Science Laboratory) | 2012 | Curiosity rover, Mars'a |
| Mars 2020 | 2021 | Perseverance rover, Mars'a |

Bu yöntemler kayda değer başarı sağlamıştır. Ancak yapısal bir sınırları vardır.

#### 3.2.1 Neden "interpolasyon problemi" — sayma argümanı

Polinom yöntemin çalışma mantığı: izlenecek yörüngenin **şekli önceden seçilir** ve zamanın bir polinomu olarak parametrize edilir. Örneğin 4. dereceden:

$$r(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4$$

Burada **5 bilinmeyen katsayı** vardır ($c_0 \ldots c_4$). Sınır koşulları yazılır:

| Koşul | Fiziksel anlamı |
|---|---|
| $r(0) = r_i$ | Şu anki konum |
| $v(0) = v_i$ | Şu anki hız |
| $r(t_f) = r_f$ | İniş noktası |
| $v(t_f) = v_f$ | İniş hızı (yumuşak iniş için $\approx 0$) |
| $a(t_f) = a_f$ | İniş anındaki ivme |

**5 bilinmeyen, 5 denklem.** Lineer sistem çözülür; tek bir cevap çıkar.

> **Kritik nokta:** Geriye optimize edilecek hiçbir şey kalmaz. Serbestlik derecesi **sıfır**dır. "Bu çözüm yakıt-optimal midir?" sorusu anlamsızdır, çünkü seçenek yoktur — zaten tek bir çözüm vardır.

Bu nedenle polinom yöntem bir **optimizasyon** değil, bir **interpolasyon** problemidir: verilen uç koşullar arasında bir eğri geçirmek.

#### 3.2.2 Cetvel analojisi

Elinizde esnek bir cetvel (şerit metal) var. İki ucunu masaya iğneyle sabitliyorsunuz ve her uçtaki eğimi de belirliyorsunuz. Cetvel artık **tek bir şekle** bürünür — başka söz hakkınız yoktur.

Şimdi masanın ortasına bir tepe koyup "cetvel bu tepeye değmesin" derseniz — yapabileceğiniz hiçbir şey yoktur. Cetvelin şekli zaten kilitlenmiştir. Ya değiyordur ya değmiyordur; ancak *sonradan kontrol* edebilirsiniz.

Optimizasyon tabanlı yaklaşımda ise cetvel yerine **serbest bir eğri** çizersiniz: sonsuz sayıda uygun eğri vardır, tepeye değmeyenleri seçersiniz ve içlerinden en az mürekkep harcayanı bulursunuz.

#### 3.2.3 Asıl açık: eşitsizlik kısıtları

Polinom yöntemin temel sınırı burada.

Sınır koşulları **eşitliktir** ($r(t_f) = r_f$) — polinom bunları doğal olarak sağlar. Ama gerçek roket kısıtlarının çoğu **eşitsizliktir**:

| Kısıt | Matematik | Anlamı |
|---|---|---|
| İtki bandı | $T_{min} \le T(t) \le T_{max}$ | Motor bu aralıkta çalışmalı |
| Tilt limiti | $\theta(t) \le \theta_{max}$ | Roket şu kadar dereceden fazla yatmasın |
| Açısal hız | $\|\omega_B(t)\|_2 \le \omega_{max}$ | Dönüş hızı sınırı |
| Glideslope | koni kısıtı | Yörünge yamaca çarpmasın |

Bunların polinom çerçevesinde **hiçbir doğal karşılığı yoktur.** Yapılabilecek tek şey: polinomu çöz → sonucu kontrol et → ihlal varsa kazançları elle ayarla → tekrar dene. Bu bir **tasarım döngüsü**dür, algoritma değil. Apollo ve MSL'de mühendisler tam olarak bunu yapmıştır; işe yaramıştır, ancak her yeni görev için baştan elle ayar gerektirmiştir.

### 3.3 Optimizasyon tabanlı yöntemlere geçiş ve IPM

Hassas iniş gereksinimleri ve karmaşık misyon kısıtları arttıkça, PDG bir **optimal kontrol problemi** olarak formüle edilmeye başlandı.

Bunu mümkün kılan gelişme **IPM (Interior Point Methods — İç Nokta Yöntemleri)** oldu [9, 10]. IPM'in kritik özelliği: konveks optimizasyon problemlerini **polinom zamanda global optimuma** garantili yakınsatabilmesi.

> Bu, mühendislik açısından çok kıymetli bir özelliktir. Bir uçuş bilgisayarında "belki yakınsar belki yakınsamaz" belirsizliği kabul edilemez; **garantili yakınsama** gerekir.

Optimizasyon algoritmalarının PDG üzerindeki etkinliği, VTVL (vertical-takeoff/vertical-landing — dikey kalkış/dikey iniş) tipi yeniden kullanılabilir uzay roketlerinin inişlerinde gösterilmiştir [8].

### 3.4 LCvx — dönüştürücü (transformative) yöntem

**LCvx (Lossless Convexification — Kayıpsız Konvekşleştirme)**, Açıkmeşe & Ploen (2007) [12] tarafından önerilmiş ve makalenin atıfta bulunduğu en kritik dönüm noktasıdır.

#### 3.4.1 Problem: itki alt sınırı neden konveks değil

3-DoF (Three Degrees of Freedom — üç serbestlik derecesi; yalnızca öteleme dinamiği + itki vektörü) PDG probleminde kontrol girdisi itki vektörü $T \in \mathbb{R}^3$'tür. Fiziksel kısıt:

$$T_{min} \le \|T\|_2 \le T_{max}$$

**Üst sınır konvekstir:** $\|T\| \le T_{max}$ bir **küre** (dolu top) tanımlar. İçindeki iki noktayı birleştiren doğru parçası hep içeride kalır. ✓

**Alt sınır konveks değildir:** $T_{min} \le \|T\|$ kümesi, ortası oyulmuş bir **küresel kabuk**tur. Kabuğun bir ucundaki $T = (+T_{min}, 0, 0)$ ile diğer ucundaki $T = (-T_{min}, 0, 0)$ noktalarını birleştirin — orta nokta $T = 0$ çıkar ve bu nokta kabuğun **dışındadır**. ✗

**Fiziksel sebep:** Sıvı yakıtlı roket motoru "biraz çalışmaz." Ya minimum itkinin üstünde çalışır, ya da tamamen kapanır — ve kapandıysa yeniden ateşlemek riskli veya imkânsızdır.

#### 3.4.2 Slack değişkeni numarası

Bir **slack variable** (gevşetme değişkeni) $\Gamma$ — skaler — eklenir. Kısıt yeniden yazılır:

$$\|T\|_2 \le \Gamma \qquad \text{(ikinci-derece koni — KONVEKS ✓)}$$
$$T_{min} \le \Gamma \le T_{max} \qquad \text{(skaler üzerinde kutu — KONVEKS ✓)}$$

Ve kütle tükenme dinamiğinde $\|T\|$ yerine $\Gamma$ kullanılır:

$$\dot m = -\alpha_{\dot m}\, \Gamma$$

Artık tüm kısıtlar konvekstir.

> ⚠️ Dikkat: Bu bir **gevşetme (relaxation)**'dir. Yeni küme orijinalinden **daha büyüktür** — çünkü $\|T\| < \Gamma$ olan durumlara da izin verir. Fiziksel anlamı: "yakıtı $\Gamma$ kadar harca ama itki olarak sadece $\|T\|$ kadar üret." Yani hayalet yakıt yakma.

#### 3.4.3 Kayıpsızlık kanıtının sezgisi

Teorem der ki: **optimal çözümde $\|T^\star\| = \Gamma^\star$ zaten kendiliğinden sağlanır.**

Sezgisel gerekçe: Diyelim optimal çözümde bir an için $\|T\| < \Gamma$ olsun. O zaman $\Gamma$'yı biraz düşürebilirdiniz — itki aynı kalır, ama **daha az yakıt yakarsınız**. Bu, çözümün optimal olduğu varsayımıyla çelişir. Demek ki optimalde eşitlik geçerlidir.

Titiz kanıt bunu **PMP (Pontryagin's Maximum Principle — Pontryagin Maksimum Prensibi)** ile yapar; costate'lerden türeyen **primer vector**'ün pozitif ölçülü bir zaman kümesinde sıfırlanamayacağını gösterir. PMP'nin rolü §5.1'de ayrıntılı işlenmiştir.

#### 3.4.4 Mağaza analojisi

Bir mağazada ürünler yalnızca **tam sayı** adet satılıyor: 1, 2 veya 3 alabilirsiniz, 1.5 alamazsınız. Bu nonconvex bir kısıttır.

Numara: "tamam, kesirli de alabilirsin" deyip kısıtı gevşetiyorsunuz. Ama fiyatlandırma öyle kurulmuş ki, **en ucuz çözüm zaten hep tam sayı çıkıyor.** Gevşetme size hiçbir şeye mal olmadı — "kayıpsız" (lossless) tam olarak bu demektir.

#### 3.4.5 Görsel: nonconvex halka → konveks koni

<svg viewBox="0 0 720 340" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">
  <style>
    .lbl { font: 13px sans-serif; fill: #cbd5e1; }
    .lbl-sm { font: 11px sans-serif; fill: #94a3b8; }
    .ttl { font: bold 14px sans-serif; fill: #e2e8f0; }
    .ok { font: bold 13px sans-serif; fill: #4ade80; }
    .bad { font: bold 13px sans-serif; fill: #f87171; }
  </style>

  <!-- ===== SOL PANEL: nonconvex halka ===== -->
  <text x="20" y="24" class="ttl">(a) Orijinal kume: T_min &lt;= ||T|| &lt;= T_max</text>
  <text x="20" y="44" class="bad">KONVEKS DEGIL</text>

  <!-- dis daire -->
  <circle cx="170" cy="190" r="110" fill="#f87171" fill-opacity="0.16" stroke="#f87171" stroke-width="2"/>
  <!-- ic daire (oyuk) -->
  <circle cx="170" cy="190" r="44" fill="#0f172a" stroke="#f87171" stroke-width="2" stroke-dasharray="5,4"/>

  <!-- kiris: iki nokta ve aralarindaki dogru -->
  <line x1="60" y1="190" x2="280" y2="190" stroke="#f59e0b" stroke-width="2.5"/>
  <circle cx="60" cy="190" r="5" fill="#f59e0b"/>
  <circle cx="280" cy="190" r="5" fill="#f59e0b"/>
  <circle cx="170" cy="190" r="6" fill="none" stroke="#f87171" stroke-width="2.5"/>
  <line x1="164" y1="184" x2="176" y2="196" stroke="#f87171" stroke-width="2.5"/>
  <line x1="176" y1="184" x2="164" y2="196" stroke="#f87171" stroke-width="2.5"/>

  <text x="24" y="182" class="lbl-sm">T_a</text>
  <text x="288" y="182" class="lbl-sm">T_b</text>
  <text x="120" y="228" class="bad">orta nokta kume DISINDA</text>
  <text x="120" y="246" class="lbl-sm">(T = 0, motor kapali)</text>
  <text x="196" y="118" class="lbl-sm">T_max</text>
  <text x="150" y="160" class="lbl-sm">T_min</text>

  <!-- ok -->
  <line x1="310" y1="190" x2="370" y2="190" stroke="#94a3b8" stroke-width="2"/>
  <polygon points="370,190 360,185 360,195" fill="#94a3b8"/>
  <text x="300" y="172" class="lbl-sm">slack degiskeni</text>
  <text x="326" y="212" class="lbl-sm">ekle</text>

  <!-- ===== SAG PANEL: konveks koni ===== -->
  <text x="400" y="24" class="ttl">(b) Gevsetilmis kume: ||T|| &lt;= G &lt;= T_max</text>
  <text x="400" y="44" class="ok">KONVEKS</text>

  <!-- eksenler -->
  <line x1="430" y1="290" x2="700" y2="290" stroke="#94a3b8" stroke-width="1.5"/>
  <line x1="565" y1="300" x2="565" y2="70" stroke="#94a3b8" stroke-width="1.5"/>
  <text x="704" y="294" class="lbl-sm">T</text>
  <text x="556" y="64" class="lbl-sm">G</text>

  <!-- konveks bolge: G >= |T| ve T_min <= G <= T_max -->
  <polygon points="565,130 495,200 495,260 635,260 635,200"
           fill="#4ade80" fill-opacity="0.18" stroke="#4ade80" stroke-width="2"/>
  <!-- G = |T| sinirlari -->
  <line x1="565" y1="130" x2="495" y2="200" stroke="#4ade80" stroke-width="2.5"/>
  <line x1="565" y1="130" x2="635" y2="200" stroke="#4ade80" stroke-width="2.5"/>
  <!-- T_min ve T_max yatay cizgileri -->
  <line x1="470" y1="200" x2="660" y2="200" stroke="#60a5fa" stroke-width="1.5" stroke-dasharray="5,4"/>
  <line x1="470" y1="260" x2="660" y2="260" stroke="#60a5fa" stroke-width="1.5" stroke-dasharray="5,4"/>
  <text x="664" y="204" class="lbl-sm">T_max</text>
  <text x="664" y="264" class="lbl-sm">T_min</text>

  <!-- kiris testi: bu sefer icinde kaliyor -->
  <line x1="510" y1="248" x2="620" y2="215" stroke="#f59e0b" stroke-width="2.5"/>
  <circle cx="510" cy="248" r="5" fill="#f59e0b"/>
  <circle cx="620" cy="215" r="5" fill="#f59e0b"/>
  <circle cx="565" cy="231.5" r="5" fill="#4ade80"/>

  <text x="430" y="315" class="ok">her kiris kume icinde ✓</text>
  <text x="586" y="150" class="lbl-sm">G = |T| kenari</text>
  <text x="586" y="166" class="lbl-sm">(optimal cozum burada)</text>
</svg>

**Şekil okuması:** (a) panelinde turuncu doğru, kümenin iki noktasını birleştiren bir kiriştir; orta noktası oyuğun içine, yani kümenin dışına düşer — konvekslik testi başarısız. (b) panelinde ise $\Gamma$ ekseni eklenerek küme $(T,\Gamma)$ uzayında bir "dondurma külahı" kesitine dönüşür; her kiriş küme içinde kalır. LCvx teoremi, optimal çözümün yeşil $\Gamma = |T|$ kenarında oturduğunu söyler — yani gevşetme bedava olmuştur.

### 3.5 LCvx'in genişletilmesi

LCvx daha sonra şu yönlerde genişletilmiştir. Her biri gerçek bir mühendislik ihtiyacına karşılık gelir:

| Genişletme                       | Kaynak | İhtiyaç                                                   |
| -------------------------------- | ------ | --------------------------------------------------------- |
| Genel nonconvex kontrol kümeleri | [13]   | Sadece itki büyüklüğü değil, daha genel kontrol kısıtları |
| Minimum-error landing            | [14]   | Hedefe tam inilemiyorsa en az sapmayla inme               |
| Thrust pointing constraints      | [15]   | İtki vektörünün belirli bir koni içinde kalması           |
| **Afin durum kısıtları**         | [16]   | Düz duvar tipi durum kısıtları                            |
| **Kuadratik durum kısıtları**    | [17]   | Elipsoit/küre tipi durum kısıtları                        |
| Nonlineer dinamikler             | [18]   | Doğrusal olmayan hareket denklemleri                      |
| Binary (ikili) kısıtlar          | [19]   | Ayrık/mantıksal seçimler                                  |

#### 3.5.1 Afin ve kuadratik durum kısıtları — ayrıntılı açıklama

Bu iki terim, kısıtın **matematiksel formuna** göre yapılan bir sınıflandırmadır.

**Afin kısıt:** durumda **lineer** artı sabit.

$$a^\top x(t) \le b$$

Geometrik olarak bir **yarı-uzay** (half-space): uzayı düz bir düzlemle ikiye bölüp "bu tarafta kal" demek.

Roket örnekleri:

| Kısıt | Matematik | Anlamı |
|---|---|---|
| Yer seviyesi | $-e_3^\top r_I(t) \le 0$ | İrtifa negatif olamaz |
| İrtifa tavanı | $e_3^\top r_I(t) \le h_{max}$ | Şu yüksekliği aşma |
| **Kuru kütle** | $-m(t) \le -m_{dry}$ | Yakıt bitmesin — **bu makalede var** (Bölüm II.B) |

**Kuadratik kısıt:** durumda **ikinci dereceden**.

$$x^\top Q x \le c$$

Geometrik olarak bir **elipsoit** veya **küre**: "bu topun içinde kal."

Roket örnekleri — hepsi bu makalede vardır:

| Kısıt | Matematik | Şekli |
|---|---|---|
| Açısal hız sınırı | $\|\omega_B(t)\|_2 \le \omega_{max}$ | Küre |
| Hız sınırı (STC) | $\|v_I(t)\|_2 \le v^{stc}_I$ | Küre |
| **Tilt açısı** | $\cos\theta_{max} \le 1 - 2(q_2^2 + q_3^2)$ | Kuadratik |

Tilt kısıtını yeniden düzenlersek:

$$q_2^2 + q_3^2 \le \frac{1 - \cos\theta_{max}}{2}$$

Yani quaternion'ın 2. ve 3. bileşenleri bir **daire** içinde kalmalıdır. Roketin ne kadar yatabileceği, tam olarak bu dairedir.

Kodda birebir bu şekilde yazılmıştır:

```python
np.sqrt((1 - np.cos(np.deg2rad(params['theta_max']))) / 2)
```

**Neden bu ayrım LCvx için önemliydi:** Durum kısıtları losslessness kanıtını zorlaştırır. Sebebi teknik ama sezgisi anlaşılır — durum kısıtının sınırına değdiğinizde costate değişkenleri **sıçrama (jump)** yapar ve PMP'nin düzgün yapısı bozulur. [16] afin durum kısıtları için, [17] kuadratik durum kısıtları için losslessness'ı genişletir.

> **Not:** "Konveks" ile "afin/kuadratik" aynı şey değildir. Ayrıntılı kavram ayrımı §5.2'dedir — bu, kolay karıştırılan bir noktadır.

### 3.6 Gerçek zamanlı uygulama ve uçuş testleri

**Customized SOCP solvers** (özelleştirilmiş İkinci Dereceden Koni Programlama çözücüleri) [20, 21] geliştirilmiştir — LCvx'in gerçek-zamanlı performansını değerlendirmek için.

**G-FOLD (Guidance for Fuel Optimal Large Divert — Yakıt-Optimal Büyük Sapma için Rehberlik)** algoritması [22–24], özellikle Masten Space Systems'ın **Xombie** adlı VTVL roketi üzerinde LCvx'i test etmek için geliştirilmiştir. Uçuş testleri [25, 26] 500 m ve 750 m sapma manevralarını kapsar.

> ⚠️ **Bu makale G-FOLD'u veya özel çözücüleri açıklamaz.** Yalnızca isim ve atıf düzeyinde geçer. Ayrıntı için [22], [23], [24] okunmalıdır. Kodda da özel çözücü implementasyonu yoktur (bkz. §5.3, §6.9).

### 3.7 3-DoF'tan 6-DoF'a geçiş zorunluluğu

Buraya kadarki tüm yöntemler **yalnızca 3-DoF**'tur — öteleme dinamiği ve itki vektörü ile ilgilenir, **tutum (attitude/rotasyon) dinamiğini** hesaba katmaz.

Gerçek roket inişinde ise:

- Roketin **tutumu** (quaternion ile temsil edilen dönüş durumu) kontrol edilmelidir
- **Açısal hız ve açısal ivme** dinamikleri devreye girer
- Bu, 3 öteleme + 3 rotasyon = **6-DoF (Six Degrees of Freedom)** demektir

6-DoF modelleme ve artan misyon kısıtı karmaşıklığı, **daha genel nonconvex problemlerin çözümünü zorunlu kılar** — bu da SCP'ye götürür.

---

## 4. Introduction — Bölüm 2/2: SCP'den D-GMSR'a

### 4.1 SCP nedir — çalışma mantığı

**SCP (Sequential Convex Programming — Ardışık Konveks Programlama)**, 3-DoF'un konveks dünyasından 6-DoF'un konveks olmayan dünyasına geçişi sağlayan yöntemdir [27–33].

**Temel fikir:** Konveks olmayan problemi doğrudan çözmeye çalışma. Onun yerine, mevcut tahminin etrafında **yerel konveks yaklaşımlar** üret ve bunları ardışık olarak çöz.

Döngü:

```
Başlangıç tahmini Z⁰
   ↓
[1] Dinamiği Z^j etrafında lineerleştir  → A_k, B_k
[2] Ortaya çıkan KONVEKS alt-problemi çöz → Z^(j+1)
[3] Yakınsadı mı? Hayır → [1]'e dön
   ↓
Çözüm (durağan nokta)
```

**Analoji:** Karanlıkta engebeli bir arazide en alçak noktayı arıyorsunuz. Tüm araziyi göremezsiniz, ama ayağınızın altındaki eğimi hissedebilirsiniz. Her adımda "buradan bakınca arazi bir düzlem gibi" varsayıp o düzlemin en alçak yönüne bir adım atarsınız. Sonra yeni noktada tekrar hissedersiniz. Yeterince küçük adımlarla dibe inersiniz.

**"Yeterince küçük adım" kritiktir** — buna **trust region** (güven bölgesi) denir. Lineerleştirme sadece yakın çevrede geçerlidir; çok büyük adım atarsanız yaklaşım çöker. Kodda bunu `w_ptr = 4096` parametresi kontrol eder (prox-linear yönteminin proksimal terimi).

#### 4.1.1 Önemli ayrım: SCP her şeyi lineerleştirmez

Yaygın bir yanlış anlama. SCP yalnızca **konveks olmayanı** lineerleştirir; konveks olanı olduğu gibi çözücüye verir.

| Bileşen | Konveks mi? | SCP ne yapar |
|---|---|---|
| **Dinamik** $\dot x = F(x,u)$ | Hayır (quaternion çarpımı, $\omega \times J\omega$, $C_{I\leftarrow B}(q)T_B$) | **Lineerleştirilir** — Jacobian $A(\tau), B(\tau)$ alınıp STM ile $A_k, B_k^\pm$ üretilir |
| Tilt açısı | Evet | **Dokunulmaz** |
| Glideslope konisi | Evet (SOC) | **Dokunulmaz** |
| Açısal hız | Evet (norm topu) | **Dokunulmaz** |
| Gimbal açıları | Evet (kutu) | **Dokunulmaz** |
| STC'ler | Hayır (mantıksal/kesikli) | D-GMSR ile pürüzsüzleştirilip cezalandırılır, sonra lineerleştirilir |

> **Sonuç:** Konveks kısıtlar her iterasyonda **tam haliyle** SOCP çözücüsüne gider. Tilt açısı kısıtı asla "yaklaşık olarak" sağlanmaz — **kesin** sağlanır. Lineerleştirilen tek şey dinamiktir.

#### 4.1.2 Bedeli: global optimum garantisi yok

| | LCvx | SCP |
|---|---|---|
| Kapsam | Sadece 3-DoF (belirli yapılar) | Genel 6-DoF, nonlineer |
| Garanti | **Global optimum** | **Stationary point / KKT noktası** |
| Yakınsama | Konveks çözücü garantisi | Prox-linear ile garantili (ama global optimum değil) |

**Dağ analojisi:** İki dağ var, biri diğerinden yüksek. Ama siz yüksek olana uzaksınız. Çevrenizde tırmanabileceğiniz en yüksek nokta, alçak olan dağ. SCP sizi oraya çıkarır ve "buradan daha yukarı gidilmiyor" der — doğrudur, ama *yerel* olarak doğrudur.

**Hangi vadide başladığınızı ne belirler?** **Initial guess** (başlangıç tahmini). Kodda (`initialize_trajectory`):

| Değişken | Başlangıç tahmini |
|---|---|
| Durum $x$ | Başlangıç–bitiş arası **doğrusal interpolasyon** |
| İtki $T$ | $\tfrac{1}{2}(T_{max} + T_{min})$, uçuş boyunca sabit |
| Gimbal açıları | Sıfır |
| $t_f$ | 21 s |

**Teselli:** Dikey iniş probleminin manzarası aşırı engebeli değildir. Çok-tepeli (multi-modal) hale getiren asıl şey **engelden kaçınma**dır — bir engelin solundan mı sağından mı geçileceği gerçekten iki ayrı vadidir ve aralarında geçiş yoktur. Düz dikey inişte böyle bir ikilem yoktur.

> **Tez için ucuz ve ikna edici doğrulama testi:** Farklı başlangıç tahminleriyle ($t_f$ = 15, 21, 30 s) aynı problemi çözüp aynı çözüme yakınsıyor mu diye bakın. Yakınsıyorsa "yerel minimum riski bu problemde pratikte düşük" diyebilirsiniz — iddia olarak değil, **ölçüm** olarak.

### 4.2 SCP'nin açtığı kapılar

Makale, SCP sayesinde ele alınabilen nonconvex özellikleri sıralar. Her biri gerçek bir fiziksel ihtiyaçtır:

| Özellik | Kaynak | Neden nonconvex |
|---|---|---|
| **Aerodinamik kuvvetler** | [34] | Sürükleme hızın karesiyle orantılı, ayrıca tutuma bağlı |
| **Free-final-time** | [35] | $t_f$ karar değişkeni olunca dinamik denklemler $t_f$ ile çarpılır → bilinmeyen × bilinmeyen |
| **Multi-phase landing** | [36] | Fazlar arası geçiş anları da bilinmeyen |
| **State-triggered constraints** | [37–39] | "Eğer X olursa Y kısıtı devreye girsin" — mantıksal, süreksiz |
| **Integer constraints** | [40, 41] | "Ya bu iniş alanı ya öteki" — ayrık seçim |

SCP ayrıca quadrotor (dört pervaneli drone) uçuşu [42–44] ve hipersonik atmosfere giriş [45–47] problemlerinde de kullanılmış, bu alanlarda gerçek-zamanlı çalışabildiği gösterilmiştir [30, 48].

### 4.3 Uçuş testine giden hat: dual quaternion → SPLICE → New Shepard

Bu, tezde "bu iş gerçekten uçuyor mu?" sorusunun cevabıdır.

**Dual quaternion** (ikili kuaterniyon), dönme ve ötelemeyi **tek bir matematiksel nesnede** birleştiren bir gösterimdir. Normal quaternion sadece dönmeyi temsil eder; konum ayrı taşınır. Dual quaternion ikisini birlikte taşır, bu da 6-DoF dinamiğini daha kompakt yazmaya izin verir.

Bu formülasyonla kurulmuş SCP tabanlı algoritma [39]:

- **SPLICE (Safe and Precise Landing – Integrated Capabilities Evolution)** — NASA'nın hassas iniş teknolojileri programı — tarafından **aday PDG algoritması** olarak seçilmiştir
- **Blue Origin New Shepard** suborbital roketinde **açık-döngü (open-loop)** konfigürasyonda uçuş testi yapılmıştır [49, 50]

> **"Açık-döngü" kritik bir nüanstır:** Algoritma uçuş sırasında çalıştırılmış ve komutları kaydedilmiştir, ama roket **gerçekten o komutlarla uçmamıştır**. Algoritmanın kararları izlenmiş, kontrol yetkisi verilmemiştir. Bu, yeni bir güdüm algoritmasını uçurmanın standart ilk adımıdır.

### 4.4 Çözücü tarafı: IPM'den PIPG'ye

Buraya kadarki tüm SCP algoritmalarında her konveks alt-problem **IPM** ile çözülüyordu. IPM güvenilirdir ama **ikinci-derece** bir yöntemdir: her iterasyonda matris çarpanlaması yapar, bu pahalıdır.

**PIPG (Proportional-Integral Projected Gradient — Oransal-İntegral İzdüşümlü Gradyan)** [51, 52] ise **birinci-derece** bir yöntemdir: sadece gradyan ve izdüşüm kullanır, matris çarpanlaması yoktur.

| Uygulama | Kaynak | Sonuç |
|---|---|---|
| 3-DoF LCvx problemine | [53] | Çözüm hızında ciddi iyileşme |
| Dual quaternion 6-DoF PDG'ye | [54] | Gerçek-zamanlı performansta belirgin artış |

> Bu, kodda ölçülen zamanlama verisiyle örtüşür: hesaplama süresinin **~%90'ı konveks alt-problem çözücüsündedir** (bkz. §6.8). Çözücüyü hızlandırmak, tüm algoritmayı hızlandırmak demektir.

### 4.5 Asıl problem: düğüm noktaları arasında ne oluyor?

**Makalenin varlık sebebi bu paragraftır.**

Yukarıda anlatılan **tüm** algoritmalarda — LCvx, SCP, dual quaternion, PIPG dahil — yol kısıtları (path constraints) **yalnızca ayrıklaştırmanın düğüm noktalarında** uygulanır. Düğümler arası (inter-sample) sağlanma **garanti edilmez**.

Somutlaştıralım. Makalenin senaryosunda $K = 15$ düğüm, $t_f \approx 21$ s:

$$\Delta t \approx \frac{21}{14} \approx 1.5 \text{ saniye}$$

Roket 50 m/s hızla düşerken 1.5 saniye = **75 metre**. Bu, sıradan bir sayısal detay değil, koca bir uçuş segmentidir.

**Yoklama analojisi:** Bir öğrencinin devamsızlığını her Pazartesi yoklama alarak takip ediyorsunuz. Öğrenci her Pazartesi derste. Yoklama defteri kusursuz. Ama Salı–Cuma hiç gelmiyor olabilir — defterde hiçbir iz yok.

**Fiziksel karşılığı ciddidir:** Glideslope konisi (yamaca çarpma koruması) düğümlerde sağlanıyor olabilir, ama iki düğüm arasında yörünge koninin dışına taşıp geri girebilir. Optimizasyon bunu **göremez**, çünkü hiç bakmaz.

> **Naif çözüm neden yetmez:** "Daha çok düğüm koy" denebilir — ama düğüm sayısı arttıkça problem boyutu ve çözüm süresi büyür, ve **hiçbir düğüm sayısı garanti vermez**; sadece riski azaltır.

### 4.6 CT-SCvx — beş bileşenli çözüm

**CT-SCvx (Continuous-Time Successive Convexification — Sürekli-Zaman Ardışık Konvekşleştirme)** [55] bu problemi kökten çözer. Beş tekniği birleştirir:

| # | Bileşen | Ne yapar | Makale bölümü |
|---|---|---|---|
| **1** | **Reformulation** (yeniden formülasyon) | Yol kısıtlarını *sürekli zamanda* sağlatır — augmented state $y$ | III.B.1 |
| **2** | **Time-dilation** (zaman genleşmesi) | Serbest-$t_f$ problemini sabit-$t_f$'ye çevirir; $s(\tau)$ eklenir | III.B.2 |
| **3** | **Exact penalty functions** | Nonconvex kısıtları maliyete ceza olarak ekler; sonlu ağırlıkta kesin çözüm | III.B.4 |
| **4** | **Multiple shooting** (çoklu atış) | Dinamiği **tam** ayrıklaştırır | III.B.3 |
| **5** | **Prox-linear method** | **Yakınsaması garantili** SCP algoritması | III.B.5 |

> ⚠️ **Beşinci maddeye dikkat:** Prox-linear yöntemi *yakınsamayı* garanti eder — algoritma bir noktada duracak, sonsuza kadar salınmayacaktır. Bu, *global optimum* garantisi **değildir**. İki farklı garanti; karıştırılmamalı ve tez metninde ayrımı net durmalıdır.

**Bir terminoloji notu:** "Sürekli zamanda sağlanma" ifadesi matematiksel olarak *"a.e. $t \in [0,t_f]$"* şeklindedir — **almost everywhere** (hemen hemen her yerde, ölçü-sıfır kümeler hariç). Pratikte "her an" demektir, ama titiz ifade budur.

### 4.7 CT-SCvx nerelerde uygulanmış

| Uygulama | Kaynak | Tez için önemi |
|---|---|---|
| **Passively-safe** yörünge optimizasyonu | [60] | Motor arızasında bile güvenli kalan yörüngeler |
| **GPU hızlandırmalı** 6-DoF PDG | [61] | Monte Carlo analizleri |
| **NMPC ile engelden kaçınma** | **[62]** | ⭐ **MPC katmanı için kritik kaynak** |
| 6-DoF uçak yaklaşma ve iniş | [63] | Runway alignment |

> ⭐ **[62] — Uzun, Elango, Kamath, Kim & Açıkmeşe, "Successive convexification for nonlinear model predictive control with continuous-time constraint satisfaction," IFAC-PapersOnLine, Vol. 58, No. 18, 2024, pp. 421–429.**
> Bu makale, CT-SCvx'i kayan ufuk (receding horizon) ile NMPC'ye dönüştürür. **Tezin MPC tarafı için bir sonraki okunacak kaynak budur.**

### 4.8 Kalan boşluk: mantıksal şartnameler

State-triggered constraints [37–39] ve integer constraints [40, 41] için formülasyonlar mevcuttur. Makalenin tespit ettiği boşluk şudur:

> **Zamansal ve mantıksal şartnameleri (temporal and logical specifications), sürekli-zamanda sağlanmalarını garanti ederek SCP çerçevesine sokan genel bir formülasyon henüz yoktur.**

**"Genel" kelimesi önemlidir.** Mevcut STC formülasyonları **tek tek** kurgulanmış çözümlerdir. İstenen: rastgele bir mantıksal ifadeyi (ve/veya/ise/her zaman/eninde sonunda) sistematik olarak optimizasyona sokabilen bir çerçeve.

### 4.9 STL — mantıksal şartnamelerin dili

**STL (Signal Temporal Logic — Sinyal Zamansal Mantığı)** [64, 65], sürekli-zamanlı sinyaller üzerinde zamansal ve mantıksal ifadeleri yazmak için geliştirilmiş biçimsel bir dildir.

Örnek şartname, düz Türkçeden STL'e:

> "İrtifa 100 metrenin altına indikten sonra, iniş anına kadar **her zaman**, hız 20 m/s'nin altında **ve** eğim açısı 5°'nin altında kalsın."

STL bunu operatörlerle yazar:

| Operatör | Sembol | Anlamı |
|---|---|---|
| Conjunction | $\wedge$ | ve |
| Disjunction | $\vee$ | veya |
| Implication | $\Rightarrow$ | ise |
| Negation | $\neg$ | değil |
| Always (globally) | $\mathbf{G}_I$ | her zaman |
| Eventually (finally) | $\mathbf{F}_I$ | eninde sonunda |
| Until | $\mathbf{U}_I$ | —e kadar |

**Sorun:** STL'in klasik robustness ölçüsü **min** ve **max** fonksiyonlarına dayanır. "Ve" = minimum al, "veya" = maksimum al. Bu, problemi bir **MIP (Mixed-Integer Problem — Karma Tamsayılı Problem)** haline getirir [67, 68]. MIP'ler kombinatoryal olarak patlayan problemlerdir — gerçek-zamanlı bir uçuş bilgisayarında düşünülemez.

### 4.10 Pürüzsüzleştirme denemeleri: soundness ve completeness

Gradyan tabanlı hızlı algoritmalar kullanabilmek için literatür min/max'ı **pürüzsüzleştirmeye** (smoothing) çalışmıştır [69–73]. Ama bu yaklaşımlar iki kritik özellikten ödün verir:

**Soundness (sağlamlık):** Parametrize edilmiş fonksiyon "şartname sağlandı" diyorsa, şartname **gerçekten sağlanmıştır**.

**Completeness (tamlık):** Şartname gerçekten sağlanıyorsa, fonksiyon **mutlaka "sağlandı" der**.

**Metal dedektör analojisi:**

| | Anlamı | İhlal edilirse |
|---|---|---|
| **Sound** | "Temiz" dediyse gerçekten temiz | Silahlı yolcu geçer → **güvenlik açığı** |
| **Complete** | Temizse mutlaka "temiz" der | Masum yolcu durdurulur → **gereksiz kısıtlama** |

**Optimizasyon karşılığı:**

- Sound değilse → **fizibil görünen ama aslında kısıt ihlal eden** çözümler üretilir (güvenilirlik sorunu)
- Complete değilse → **aslında geçerli olan çözümler reddedilir** (optimallik kaybı, hatta yanlış "çözüm yok" hatası)

### 4.11 Locality & masking — daha sinsi bir problem

Min/max kullanımının ayrı bir sorunu daha vardır [74]: **problem fizibil olduğu halde optimizasyon çözüm bulamayabilir.**

**Masking (maskeleme):** Beş kısıtınız var, "ve" ile bağlı. Minimum alıyorsunuz. Gradyan sadece **en kötü** kısıttan geliyor. Diğer dördü hakkında optimizasyon **hiçbir bilgi almıyor** — türevleri sıfır.

**Takım analojisi:** Beş kişilik bir takımda sadece en zayıf üyenin skoru sayılıyor. Diğer dördü ne yaparsa yapsın geri bildirim alamıyor — ilerlediklerini ya da gerilediklerini bilemiyorlar. Ve en zayıf üye değiştiğinde geri bildirim aniden başka birine sıçrıyor.

**Locality (yerellik):** Bu ani sıçrama, fonksiyonun pürüzsüz olmaması demektir — gradyan süreksizdir.

Sonuç: yerel optimizasyon algoritmaları min/max ile iyi çalışmaz.

### 4.12 D-GMSR — makalenin kullandığı çözüm

**D-GMSR (Discrete Generalized Mean-based Smooth Robustness — Ayrık Genelleştirilmiş Ortalama Tabanlı Pürüzsüz Robustluk Ölçüsü)** [75], min/max yerine **genelleştirilmiş ortalamalar** (ağırlıklı geometrik ortalama, ağırlıklı kuvvet ortalaması) kullanır.

**Neden işe yarar:** Ortalama, **tüm** elemanlardan katkı alır. Beş kısıdın hepsi gradyana katkıda bulunur — maskeleme yok. Ve ortalama fonksiyonları pürüzsüzdür — yerellik problemi yok.

Makalenin Tablo 1'i — D-GMSR'ın diğer robustness ölçüleriyle karşılaştırması:

| Özellik | [65] | [69] | [71] | [74] | [72] | **D-GMSR** |
|---|---|---|---|---|---|---|
| $C^1$-pürüzsüzlük | ✗ | ✓ | ✓ | ✗ | ✗ | **✓** |
| Soundness | ✓ | ○ | ✓ | ✓ | ✓ | **✓** |
| Completeness | ✓ | ○ | ○ | ✓ | ✓ | **✓** |
| Monotonicity | ✓ | ✓ | ✓ | ✓ | ✗ | **✓** |
| Locality & Masking | ✗ | △ | △ | ✓ | ✓ | **✓** |

○ = yalnızca çok büyük pürüzsüzleştirme parametreleri için sağlanır
△ = yalnızca küçük pürüzsüzleştirme parametreleri için sağlanır

> **Tablonun okuması:** Diğer beş yöntemin hiçbiri beş özelliği birden sağlamaz. Bazıları soundness/completeness'ı "çok büyük parametre" için, bazıları locality direncini "küçük parametre" için sağlar — yani **birbiriyle çelişen ayarlar** gerektirirler. D-GMSR bu çelişkiyi ortadan kaldırır.

**$C^1$-pürüzsüzlük** = fonksiyonun birinci türevi var ve sürekli. Gradyan tabanlı optimizasyonun asgari şartı.

### 4.13 Makalenin konumu ve organizasyonu

| Bölüm | İçerik |
|---|---|
| II | 6-DoF PDG problemi: dinamik, kısıtlar, sınır koşulları, compound STC'ler |
| III | D-GMSR parametrizasyonu + CT-SCvx çözüm çerçevesi |
| IV | Sayısal sonuçlar (Tablo 3, Fig. 1–2) |
| V | Sonuç ve gelecek çalışmalar |

---

## 5. Derinleştirilmiş konular

Bu bölüm, Introduction'da yalnızca değinilen ama tam anlaşılması kritik olan dört konuyu ayrıntılandırır.

### 5.1 PMP — gösterge mi, kanıt aracı mı?

**Kısa cevap: gösterge değil, kanıt aracıdır.** Ama teoremin varsayımları sağlandığı sürece, gevşetilmiş problemin optimal çözümü orijinal problemin de optimal çözümüdür.

#### 5.1.1 PMP nedir

**PMP (Pontryagin's Maximum Principle)**, optimal kontrol problemlerinde optimalliğin **gerekli koşullarını** veren teoremdir.

**Analoji:** Lisede öğrenilen *"minimumda $f'(x) = 0$"* koşulunu düşünün. Bu bir **gerekli** koşuldur — her minimumda türev sıfırdır. Ama **yeterli** değildir: türevin sıfır olduğu her nokta minimum değildir (maksimum veya eyer noktası da olabilir).

PMP, bunun kontrol problemlerine genellemesidir. Fark: burada bilinmeyen bir *sayı* değil, bir *fonksiyondur* ($u(t)$, yani tüm zaman boyunca kontrol geçmişi). PMP, bu fonksiyonun optimal olabilmesi için sağlaması gereken koşulları verir ve bunu yaparken **costate** (eş-durum, $\lambda(t)$) adı verilen yardımcı değişkenleri tanıtır.

#### 5.1.2 Lossless kanıtında nasıl kullanılır

```
1. Gevşetilmiş (konveks) problemi yaz
2. PMP'yi uygula → costate denklemleri çıkar
3. Costate'lerin yapısını incele
4. Göster ki: optimal çözümde ‖T‖ = Γ olmak ZORUNDA
5. Dolayısıyla gevşetme "sıkı" (tight) → kayıpsız
```

4. adımın çekirdeği: costate'lerden türeyen **primer vector** adlı bir vektörün, pozitif uzunlukta bir zaman aralığında sıfır olamayacağı gösterilir. Bu sıfır olmayış, itki büyüklüğünün sınırda oturmasını zorlar.

#### 5.1.3 Kritik ayrım: ne zaman kullanılır

| | Nerede | Ne zaman |
|---|---|---|
| **PMP** | Kâğıt üzerinde, makalede | **Bir kez**, algoritma yazılmadan önce |
| **Algoritma çalışırken** | Uçuş bilgisayarı | PMP'ye **hiç bakılmaz** |

> Uçuş sırasında hiçbir şey kontrol edilmez. Teorem bir kez ispatlanmıştır, güven oradan gelir. PMP bir "indikatör" değil, tasarım zamanında verilmiş bir garantidir.

#### 5.1.4 Ama pratikte sayısal doğrulama yapılabilir

Çözüm geldikten sonra her düğümde şu karşılaştırma yapılmalıdır:

$$\|T^\star_k\| \stackrel{?}{=} \Gamma^\star_k$$

Eşitse gevşetme sıkıydı. Bir yerde $\|T^\star\| < \Gamma^\star$ çıkarsa, gevşetme o noktada "gevşek kalmış" demektir — yani teoremin varsayımlarından biri sağlanmıyordur.

> ⚠️ **Varsayımlar önemlidir.** Losslessness kanıtları koşulsuz değildir. Kontrol edilebilirlik, tekil yay (singular arc) bulunmaması gibi teknik şartlar gerekir. Bunlar bozulursa losslessness bozulur. "Lossless kanıtları kırılgandır" derken kastedilen budur.

#### 5.1.5 Bu makale için önemli hatırlatma

> ⚠️ **Bu tartışmanın tamamı LCvx'e aittir. Uzun–Açıkmeşe–Carson makalesi LCvx kullanmaz, SCP kullanır.** SCP'de PMP yoktur, lossless kanıtı yoktur. Oradaki garanti tamamen farklıdır: prox-linear yönteminin **yakınsama garantisi**.

| | LCvx | SCP (bu makale) |
|---|---|---|
| Garanti aracı | PMP + lossless teoremi | Prox-linear yakınsama teoremi |
| Ne garanti edilir | Global optimum | Durağan noktaya (KKT) yakınsama |

Makale LCvx'i yalnızca **literatür arka planı** olarak anar; kanıtı [12], [13], [15]–[19] kaynaklarına havale eder. Lossless convexification kanıtı için bu kaynaklar ayrıca okunmalıdır.

### 5.2 Konvekslik vs afin/kuadratik — kolay karıştırılan kavram ayrımı

Bu iki kavram **farklı kategorilerdir**. Biri kümenin geometrisiyle, diğeri kısıtın yazım biçimiyle ilgilidir.

#### 5.2.1 Konvekslik testi (tek ve evrensel)

> Bir küme konvekstir ⟺ kümenin içindeki **herhangi iki noktayı** düz bir çizgiyle birleştirdiğinizde, çizginin **her noktası** da kümenin içindedir.

**Analoji:** Bir odada iki kişi duruyor. Birbirlerini görebiliyorlarsa (aralarındaki düz çizgi hiçbir duvara çarpmıyorsa) oda konvekstir. L-şeklinde bir odada köşedeki iki kişi birbirini göremeyebilir — o zaman konveks değildir.

**Bu tanım hiçbir yerde "afin mi kuadratik mi" demez.** Şekle bakar, formüle değil.

#### 5.2.2 Afin ve kuadratik, bu testi geçen (veya geçmeyen) iki örnek aile

| Form | Genel yazım | Şekli | Konveks mi? |
|---|---|---|---|
| **Afin** | $a^\top x \le b$ | Düz duvar (yarı-uzay) | **Her zaman** evet |
| **Kuadratik** | $x^\top Q x \le c$ | Elipsoit / eyer yüzeyi | **Bazen** — $Q$'ya bağlı |

**Afin satırı:** Düz bir duvarın iki tarafı asla "L şekli" oluşturmaz, o yüzden afin kısıtlar **koşulsuz** konvekstir.

**Kuadratik satırı şartlıdır:** $Q$ **pozitif yarı-tanımlı (positive semi-definite)** ise küme bir elipsoit/top gibi dışbükey çıkar → konveks. $Q$'nun negatif özdeğerleri varsa küme bir **eyer** veya **saat kumu** şeklinde çıkar → konveks **değil**.

**Somut doğrulama — makaledeki tilt kısıtı:**

$$q_2^2 + q_3^2 \le \frac{1-\cos\theta_{max}}{2}$$

Sol taraf $q_2^2 + q_3^2$ — bir **dolu daire (disk)** tanımlar, $Q = I$ (birim matris, pozitif tanımlı). Disk konvekstir: iki nokta seçin, aralarındaki çizgi hep diskin içinde kalır. ✓ **Konveks kuadratik kısıt.**

**Karşı örnek:** $q_2^2 - q_3^2 \le c$ olsaydı — bu bir **hiperbol** tanımlardı, iki ayrı kol. İki kolun üzerinden birer nokta seçin; aralarındaki çizgi kollar arasındaki boşluktan geçer → küme dışında. ✗ **Konveks olmayan kuadratik kısıt.**

> **Sonuç:** "Kuadratik" yazılan her şey otomatik konveks çıkmaz. Konvekslik $Q$ matrisinin yapısına bağlıdır.

#### 5.2.3 Üçüncü aile: konik kısıtlar

Konveks kısıtlar afin ve kuadratikten ibaret **değildir.** Makalede üçüncü bir aile daha vardır:

| Aile | Örnek (makaleden) | Şekli |
|---|---|---|
| **Afin** | $m(t) \ge m_{dry}$ | Düz duvar |
| **Konveks kuadratik** | Tilt açısı kısıtı | Disk |
| **Konik (SOC)** | Glideslope: $\tan\gamma_{max}\|[e_1\,e_2]^\top r_I\|_2 \le e_3^\top r_I$ ; açısal hız: $\|\omega_B\|_2 \le \omega_{max}$ | Koni / küre |

Konik kısıtlar afin de değil kuadratik de değildir — norm içerirler. Ama **bunlar da konvekstir.** SOCP çözücülerinin var olma sebebi tam olarak bu üçüncü ailedir.

#### 5.2.4 Toparlama

$$\text{Konveks} \;=\; \{\text{Afin}\} \;\cup\; \{\text{konveks Kuadratik}\} \;\cup\; \{\text{Konik}\} \;\cup\; \{\text{diğer konveks formlar}\}$$

> Afin ve kuadratik, konveks kümenin **tanımı değil, iki alt kümesidir.**

#### 5.2.5 Görsel: konveks aileler ve bir karşı örnek

<svg viewBox="0 0 720 210" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">
  <style>
    .t { font: bold 12px sans-serif; fill: #e2e8f0; }
    .s { font: 11px sans-serif; fill: #94a3b8; }
    .g { font: bold 11px sans-serif; fill: #4ade80; }
    .r { font: bold 11px sans-serif; fill: #f87171; }
  </style>

  <!-- 1: AFIN -->
  <text x="20" y="18" class="t">Afin</text>
  <text x="20" y="34" class="s">a'x &lt;= b</text>
  <rect x="20" y="44" width="140" height="120" fill="none" stroke="#475569" stroke-width="1"/>
  <polygon points="20,44 160,44 160,110 20,130" fill="#4ade80" fill-opacity="0.2"/>
  <line x1="20" y1="130" x2="160" y2="110" stroke="#4ade80" stroke-width="2.5"/>
  <line x1="46" y1="70" x2="132" y2="96" stroke="#f59e0b" stroke-width="2"/>
  <circle cx="46" cy="70" r="4" fill="#f59e0b"/>
  <circle cx="132" cy="96" r="4" fill="#f59e0b"/>
  <text x="20" y="182" class="g">her zaman konveks</text>

  <!-- 2: KUADRATIK (konveks) -->
  <text x="195" y="18" class="t">Kuadratik (Q &gt; 0)</text>
  <text x="195" y="34" class="s">q2^2 + q3^2 &lt;= c</text>
  <rect x="195" y="44" width="140" height="120" fill="none" stroke="#475569" stroke-width="1"/>
  <circle cx="265" cy="104" r="50" fill="#4ade80" fill-opacity="0.2" stroke="#4ade80" stroke-width="2.5"/>
  <line x1="232" y1="78" x2="298" y2="126" stroke="#f59e0b" stroke-width="2"/>
  <circle cx="232" cy="78" r="4" fill="#f59e0b"/>
  <circle cx="298" cy="126" r="4" fill="#f59e0b"/>
  <text x="195" y="182" class="g">konveks (disk)</text>

  <!-- 3: KONIK -->
  <text x="370" y="18" class="t">Konik (SOC)</text>
  <text x="370" y="34" class="s">tan(g)*||r_xy|| &lt;= r_z</text>
  <rect x="370" y="44" width="140" height="120" fill="none" stroke="#475569" stroke-width="1"/>
  <polygon points="440,150 385,58 495,58" fill="#4ade80" fill-opacity="0.2" stroke="#4ade80" stroke-width="2.5"/>
  <line x1="415" y1="88" x2="470" y2="112" stroke="#f59e0b" stroke-width="2"/>
  <circle cx="415" cy="88" r="4" fill="#f59e0b"/>
  <circle cx="470" cy="112" r="4" fill="#f59e0b"/>
  <text x="370" y="182" class="g">konveks (koni)</text>

  <!-- 4: KONVEKS DEGIL -->
  <text x="545" y="18" class="t">Karsi ornek</text>
  <text x="545" y="34" class="s">T_min &lt;= ||T||</text>
  <rect x="545" y="44" width="140" height="120" fill="none" stroke="#475569" stroke-width="1"/>
  <circle cx="615" cy="104" r="50" fill="#f87171" fill-opacity="0.18" stroke="#f87171" stroke-width="2.5"/>
  <circle cx="615" cy="104" r="20" fill="#0f172a" stroke="#f87171" stroke-width="2" stroke-dasharray="4,3"/>
  <line x1="565" y1="104" x2="665" y2="104" stroke="#f59e0b" stroke-width="2"/>
  <circle cx="565" cy="104" r="4" fill="#f59e0b"/>
  <circle cx="665" cy="104" r="4" fill="#f59e0b"/>
  <line x1="610" y1="99" x2="620" y2="109" stroke="#f87171" stroke-width="2.5"/>
  <line x1="620" y1="99" x2="610" y2="109" stroke="#f87171" stroke-width="2.5"/>
  <text x="545" y="182" class="r">KONVEKS DEGIL</text>
</svg>

Turuncu doğrular konvekslik testidir: kümenin iki noktası arasındaki kiriş. İlk üç panelde kiriş küme içinde kalır (✓), dördüncüde oyuktan geçer (✗).

### 5.3 Özel SOCP çözücüleri — makalede ve kodda yok

Doğrulanmış tespit:

**Makalede:** Yalnızca isim düzeyinde geçer.
- [20], [21] → gömülü, gerçek-zamanlı SOCP çözücüleri (kod üretimi tabanlı)
- [51]–[54] → PIPG

Hiçbirinin algoritması açıklanmaz.

**Kodda:** Arama sonucu:

| Terim | Eşleşme sayısı |
|---|---|
| `PIPG` | **0** |
| `MOSEK` | 1 — `problem.solve(solver='MOSEK')` |
| `CLARABEL` | 1 — `problem.solve(solver='CLARABEL')` |
| İç nokta yöntemi implementasyonu | **0** |

> Kod **hazır çözücü** kullanır, kendi çözücüsünü yazmaz. CVXPY ile problem kurulur, MOSEK veya CLARABEL'e gönderilir. MOSEK lisansı olanlar kodu doğrudan çalıştırabilir; kodda MOSEK satırı aktiftir.

#### 5.3.1 "Customized" ne demek

Özel çözücülerin genel çözücülerden farkı yalnızca hız değil, **öngörülebilirliktir**.

| | Genel çözücü (MOSEK) | Özel çözücü |
|---|---|---|
| Bellek | Dinamik ayırma | Statik, sabit |
| Dallanma | Problem yapısına göre | Yok |
| Çalışma süresi | Değişken | **Her çağrıda aynı** |
| Uçuş bilgisayarı uygunluğu | Hayır | Evet |

Sertifikasyon için sabit çalışma süresi şarttır. Bu, tezde bir gelecek çalışma maddesi olabilir; ilk hedef (basit dikey iniş) için gerekli değildir.

### 5.4 Sürekli-zaman garantisi — tam mekanizma

Makalenin en zarif kısmı. Adım adım.

#### 5.4.1 Adım 1 — Ceza fonksiyonu tanımla

$$q_c(0, g(x)) = \max(0, g(x))^2$$

Davranışı:

| Durum | $g(x)$ | $\max(0,g)^2$ |
|---|---|---|
| Kısıt sağlanıyor | $\le 0$ | **0** |
| Kısıt ihlal ediliyor | $> 0$ | $> 0$ |

İki kritik özellik: **her zaman negatif değildir**, ve **tam olarak sıfır olması ancak kısıt sağlanırsa mümkündür**.

#### 5.4.2 Adım 2 — Yol boyunca integre et

$$\int_0^{t_f} \Big[\underbrace{\textstyle\mathbf{1}^\top q_c(0, g_x)}_{\text{durum kısıtları}} + \underbrace{\textstyle\mathbf{1}^\top q_c(0, g_u)}_{\text{kontrol kısıtları}} + \underbrace{\textstyle\mathbf{1}^\top h_{stc}}_{\text{STC'ler}}\Big]\, dt = 0$$

#### 5.4.3 Adım 3 — Matematiksel çekirdek

> **Negatif olmayan bir fonksiyonun integrali sıfırsa, fonksiyonun kendisi hemen hemen her yerde sıfırdır.**

**Sezgisi:** İntegrand hiçbir yerde negatif olamadığı için, hiçbir pozitif bölge başka bir bölge tarafından "telafi edilemez". Tek bir anda bile pozitif olsa (ve makul düzgünlükte olsa), integral pozitif çıkar.

**Yağmur ölçer analojisi:** Bahçenize bir kap koyuyorsunuz. Yağmur yağınca su birikiyor, yağmadığında seviye sabit kalıyor. Ay sonunda kaba bakıyorsunuz: **boş**. Bu, o ay boyunca *hiç* yağmur yağmadığı anlamına gelir — sadece baktığınız günlerde değil, her an. Çünkü su buharlaşıp geri gidemez (negatif yağmur yoktur).

> **Makalenin Remark 3'ü:** D-GMSR ile parametrize edilmiş STC fonksiyonları **zaten negatif değildir**, o yüzden onlara ek dış ceza (exterior penalty) fonksiyonu uygulamaya gerek yoktur.

#### 5.4.4 Adım 4 — İntegrali duruma çevir

**Sorun:** Elimizde bir fonksiyonun integrali var ve bunun sıfır olmasını kısıt olarak dayatmak istiyoruz. Ama optimizasyon çözücüsü **integral alamaz** — yalnızca sonlu sayıda değişken ve sonlu sayıda kısıt görebilir. Sürekli bir fonksiyonun tamamının integrali gibi "sonsuz boyutlu" bir nesne doğrudan verilemez.

**Odometre analojisi:** Arabanızın kilometre sayacı, toplam yolu matematiksel olarak hızın integrali olarak taşır:

$$\text{Toplam yol} = \int_0^{t_f} v(t)\, dt$$

Ama arabanız bunu **böyle hesaplamaz**. Ayrı bir "integral alma" işlemi yapmaz. Onun yerine, sayaç kendi başına bir **durum**dur ve şu kuralla ilerler:

$$\dot{(\text{sayaç})}(t) = v(t), \qquad \text{sayaç}(0) = 0$$

Sayacın "hızı", arabanın hızına eşittir. $t_f$ anında sayaca bakarsınız — orada duran sayı, otomatik olarak integral sonucudur. **Hiçbir ayrı integral hesabı yapılmamıştır.**

**Bizim durumumuzda birebir aynı numara:** 15. durum $y$ tanımlanır ve "hızı" integrand'a eşitlenir:

$$\dot y(t) \;:=\; \mathbf{1}^\top q_c\big(0, g_x(x(t))\big) + \mathbf{1}^\top q_c\big(0, g_u(u(t))\big) + \mathbf{1}^\top h_{stc}\big(x(t), u(t)\big)$$

$$y(0) = y(t_f)$$

Kodda birebir karşılığı:

```python
f = f.at[14].set(
    tilt_ang_cons + ang_vel_cons + gs_cons          # ← durum kısıtı ihlalleri (g_x)
    + gs_stc + los_stc + tilt_ang_stc + spds_stcs    # ← STC ihlalleri (h_stc)
    + gimbal_ang_stc + thrust_stc_f + thrust_stc_i   # ← kontrol kısıtı ihlalleri (g_u)
)
```

#### 5.4.5 Adım 5 — İşin sırrı: neden aralar görülüyor

$y$ artık bir **durum**dur. Ve durumlar, dinamiği entegre eden **aynı sayısal entegratör** tarafından taşınır. Kodda `rk4_steps_dyn = 20` — her düğüm aralığı 20 alt-adıma bölünüp RK4 ile entegre edilir.

| | Değer |
|---|---|
| Düğüm sayısı $K$ | 15 |
| Düğüm aralığı | ~1.5 s |
| Aralık başına RK4 alt-adımı | 20 |
| **Etkin kısıt örnekleme aralığı** | **~0.075 s** |

Kısıtlar 1.5 saniyede bir değil, **0.075 saniyede bir** yoklanır — 20 kat daha sık. İhlaller $y$'de birikip son kısıta yansır.

Akış:

```
Düğüm k'da: x = [m, r, v, q, ω, y]   (14 fiziksel + 1 sayaç)
    ↓ RK4, 20 alt-adım (her biri ~0.075 s)
    Her alt-adımda: ẋ = F(x,u)        hesaplanır
                    ẏ = integrand(x,u) hesaplanır  ← aynı anda, aynı mekanizmayla
    ↓
Düğüm k+1'de: x = [m', r', v', q', ω', y']
```

> **$y$'nin hiçbir özel muamelesi yoktur.** Diğer durumlar nasıl fizik denklemleriyle ileri taşınıyorsa, $y$ de kendi (ihlal) denklemiyle aynı şekilde taşınır. Fark: diğerleri fiziksel büyüklükleri, $y$ ise "ne kadar kural çiğnedim" sayacını temsil eder.

> ⚠️ **Dürüst not:** Matematiksel ifade kesindir ("sürekli zamanda sağlanır"), ama sayısal gerçekleşme entegratörün doğruluğu kadar iyidir. Sonsuz çözünürlük değil, çok yüksek çözünürlük. Tez metninde bu ayrım belirtilmelidir.

#### 5.4.6 Adım 6 — Neden tam eşitlik değil de $\epsilon$?

Makale Eq. (7c)'de eşitliği gevşetir:

$$^yE(\tilde x_{k+1} - \tilde x_k) \le \epsilon_{LICQ}$$

Kodda: `beta_ctcs = 1e-4`, ve kısıt şu satırda:

```python
vehicle_cons += [X[-1:, 1:] - X[-1:, 0:-1] <= params['beta_ctcs']]
```

**Neden gevşetmek gerekir?** $\max(0, g)^2$ fonksiyonu $g = 0$ noktasında **düzdür** — hem değeri hem türevi sıfır. Tam eşitlik dayatıldığında, çözücünün gördüğü kısıt gradyanı sıfır çıkar ve **LICQ (Linear Independence Constraint Qualification — Lineer Bağımsızlık Kısıt Niteliği)** bozulur. Çözücü o noktada hangi yöne gideceğini bilemez.

**Analoji:** Dümdüz bir vadi tabanında gözü kapalı duruyorsunuz. "Aşağı hangi taraf?" diye soruyorsunuz. Her taraf aynı. Eğim yok.

$10^{-4}$ kadar pay bırakılınca gradyan tekrar anlamlı hale gelir.

**Bedeli:** Artık $10^{-4}$ kadar ihlale izin verilir. Telafi için kısıtların kendisi **sıkılaştırılır** — kodda `alpha_*` çarpanları ($\delta_{LICQ}$ değerleri):

| Parametre | Değer | Etki |
|---|---|---|
| `alpha_theta_stc_cons` | 0.92 | Tilt kısıtı %8 daha sıkı |
| `alpha_gs_stc_cons` | 0.92 | Glideslope %8 daha sıkı |
| `alpha_spd_stc_cons` | 0.95 | Hız kısıtı %5 daha sıkı |
| `alpha_omega_stc_cons` | 0.99 | Açısal hız %1 daha sıkı |
| `alpha_los_stc_cons` | 0.98 | Line-of-sight %2 daha sıkı |
| `alpha_T_min` | 1.02 | Minimum itki %2 daha yüksek |
| `alpha_alt_stc_trig` | 1.10 | Tetikleme irtifası %10 daha erken |

> Mantık: "Biraz ihlale izin veriyorum, o yüzden kısıtı biraz daha içeriden çiziyorum."
>
> ⚠️ **Makale bu sayıları vermez.** Yalnızca "her $\epsilon_{LICQ}$ için karşılık gelen bir $\delta_{LICQ}$ vardır" der. Somut değerler yalnızca koddadır.

#### 5.4.7 Özet zincir

$$\underbrace{\max(0,g)^2}_{\text{negatif olmayan ceza}} \;\to\; \underbrace{\int_0^{t_f}(\cdot)\,dt = 0}_{\text{integral sıfır}} \;\to\; \underbrace{\dot y = (\cdot),\; y(0)=y(t_f)}_{\text{durum olarak taşı}} \;\to\; \underbrace{\text{RK4 alt-adımları}}_{\text{aralar örnekleniyor}}$$

#### 5.4.8 Görsel: düğüm-yalnız kontrol vs sürekli-zaman garantisi

<svg viewBox="0 0 720 430" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">
  <style>
    .tt { font: bold 13px sans-serif; fill: #e2e8f0; }
    .ll { font: 12px sans-serif; fill: #cbd5e1; }
    .ss { font: 10px sans-serif; fill: #94a3b8; }
    .rr { font: bold 11px sans-serif; fill: #f87171; }
    .gg { font: bold 11px sans-serif; fill: #4ade80; }
  </style>

  <!-- ===== UST PANEL: yorunge ve kisit siniri ===== -->
  <text x="20" y="20" class="tt">(a) Yorunge ve kisit siniri</text>

  <!-- kisit siniri -->
  <line x1="70" y1="110" x2="660" y2="110" stroke="#4ade80" stroke-width="2.5" stroke-dasharray="7,4"/>
  <text x="666" y="114" class="gg">g=0</text>
  <text x="70" y="102" class="ss">guvenli bolge asagisi</text>

  <!-- dugum noktalari (x ekseni) -->
  <line x1="70" y1="190" x2="660" y2="190" stroke="#475569" stroke-width="1.5"/>

  <!-- yorunge: dugumlerde guvenli, arada ihlal -->
  <path d="M 70,150 Q 145,148 220,145 Q 295,142 370,143
           C 420,143 440,78 480,78
           C 520,78 540,143 590,145 Q 625,146 660,150"
        fill="none" stroke="#60a5fa" stroke-width="2.5"/>

  <!-- ihlal bolgesi (kisit sinirinin ustu) -->
  <path d="M 425,110 C 440,80 460,78 480,78 C 500,78 520,80 535,110 Z"
        fill="#f87171" fill-opacity="0.3" stroke="#f87171" stroke-width="1.5"/>
  <text x="400" y="66" class="rr">IHLAL — iki dugum arasinda</text>

  <!-- dugum noktalari -->
  <g fill="#f59e0b">
    <circle cx="70"  cy="150" r="5"/>
    <circle cx="220" cy="145" r="5"/>
    <circle cx="370" cy="143" r="5"/>
    <circle cx="590" cy="145" r="5"/>
    <circle cx="660" cy="150" r="5"/>
  </g>
  <g stroke="#f59e0b" stroke-width="1" stroke-dasharray="3,3">
    <line x1="70"  y1="150" x2="70"  y2="190"/>
    <line x1="220" y1="145" x2="220" y2="190"/>
    <line x1="370" y1="143" x2="370" y2="190"/>
    <line x1="590" y1="145" x2="590" y2="190"/>
    <line x1="660" y1="150" x2="660" y2="190"/>
  </g>
  <text x="60"  y="206" class="ss">t_k</text>
  <text x="208" y="206" class="ss">t_k+1</text>
  <text x="358" y="206" class="ss">t_k+2</text>
  <text x="578" y="206" class="ss">t_k+3</text>

  <!-- RK4 alt adimlari -->
  <g stroke="#a78bfa" stroke-width="1.2">
    <line x1="385" y1="186" x2="385" y2="194"/><line x1="400" y1="186" x2="400" y2="194"/>
    <line x1="415" y1="186" x2="415" y2="194"/><line x1="430" y1="186" x2="430" y2="194"/>
    <line x1="445" y1="186" x2="445" y2="194"/><line x1="460" y1="186" x2="460" y2="194"/>
    <line x1="475" y1="186" x2="475" y2="194"/><line x1="490" y1="186" x2="490" y2="194"/>
    <line x1="505" y1="186" x2="505" y2="194"/><line x1="520" y1="186" x2="520" y2="194"/>
    <line x1="535" y1="186" x2="535" y2="194"/><line x1="550" y1="186" x2="550" y2="194"/>
    <line x1="565" y1="186" x2="565" y2="194"/><line x1="580" y1="186" x2="580" y2="194"/>
  </g>
  <text x="392" y="224" style="font:10px sans-serif;fill:#a78bfa">RK4 alt-adimlari (~0.075 s) — ihlali GORUR</text>

  <text x="70" y="248" class="rr">Dugum-yalniz kontrol: 5 dugumun hepsi guvenli tarafta → optimizasyon ihlali GORMEZ</text>

  <!-- ===== ALT PANEL: y sayaci ===== -->
  <text x="20" y="288" class="tt">(b) y sayaci — birikmis ihlal</text>

  <line x1="70" y1="400" x2="660" y2="400" stroke="#475569" stroke-width="1.5"/>
  <line x1="70" y1="400" x2="70" y2="305" stroke="#475569" stroke-width="1.5"/>
  <text x="44" y="312" class="ss">y</text>
  <text x="664" y="404" class="ss">t</text>

  <!-- y egrisi: duz, sonra tirmanis, sonra duz -->
  <path d="M 70,395 L 420,395 C 450,395 470,352 480,340 C 495,326 515,318 540,318 L 660,318"
        fill="none" stroke="#f87171" stroke-width="2.5"/>
  <line x1="70" y1="395" x2="660" y2="395" stroke="#4ade80" stroke-width="1.5" stroke-dasharray="4,4"/>

  <text x="140" y="388" class="ss">sabit (ihlal yok)</text>
  <text x="440" y="300" class="rr">TIRMANIS</text>
  <text x="560" y="310" class="ss">sabit ama YUKSEK seviyede</text>

  <!-- fark oku -->
  <line x1="640" y1="395" x2="640" y2="318" stroke="#f59e0b" stroke-width="2"/>
  <polygon points="640,318 635,328 645,328" fill="#f59e0b"/>
  <polygon points="640,395 635,385 645,385" fill="#f59e0b"/>
  <text x="520" y="424" style="font:bold 11px sans-serif;fill:#f59e0b">y(t_f) - y(0) &gt; 0  →  kisit REDDEDER</text>
</svg>

**Şekil okuması:** (a) panelinde mavi yörünge, beş turuncu düğümün **hepsinde** güvenli taraftadır — düğüm-yalnız kontrol bu çözümü kabul eder. Ama iki düğüm arasında yeşil kısıt sınırını aşar (kırmızı bölge). Mor çentikler RK4 alt-adımlarıdır ve bu ihlali **görürler**. (b) panelinde $y$ sayacı: ihlal yokken düz, ihlal boyunca tırmanıyor, sonra yüksek seviyede sabitleniyor — asla geri düşmüyor. Son kısıt $y(t_f) - y(0) \le \epsilon$ bu çözümü **reddeder**.

#### 5.4.9 $y$ değişkeninin doğası — sık sorulan noktalar

**(1) $y$ yapay bir değişken midir?**

**Evet.** $y$, roketin fiziğinde doğal olarak var olan bir büyüklük **değildir**. Kütle, konum, hız, quaternion, açısal hız — bunlar roketin gerçekten sahip olduğu niceliklerdir; $y$ olmasa da roket uçar. $y$ sonradan **icat edilip** durum vektörüne **eklenir**, sırf matematiksel bir amaç için.

Literatürdeki adı **augmented state** (genişletilmiş/eklenmiş durum). Makalenin kendi ifadesi:

> *"The system dynamics are then augmented with a new state variable $y \in \mathbb{R}_+$"*

Kodda durum vektörü fiziksel olarak 14 boyutlu olması gerekirken 15 boyutludur — o fazladan boyut, eklenen sayaçtır.

**Sınıf analojisi:** Bir sınıfta öğretmen gerçek bir ders işliyor (fizik: roketin gerçek hareketi), ama yan tarafta "bugün kaç kere kural ihlali oldu" diye tutulan bir tebeşir sayacı var ($y$). Sayaç, dersin fiziğinin parçası değil — bizim tuttuğumuz muhasebe defteri. Defterin işleyiş kuralını da biz koyuyoruz.

**(2) $y$ azalabilir mi?**

**Hayır.** Bu, $y$'yi fiziksel bir konumdan ayıran en önemli özelliktir.

$$\dot y(t) = \underbrace{\text{hepsi } \max(0,\cdot)^2 \text{ formunda}}_{\ge 0} \;\ge\; 0 \quad \text{HER ZAMAN}$$

$\max(0,\cdot)^2$ asla negatif çıkamaz. Yani $y$'nin "hızı" hiçbir zaman negatif olmaz → **$y$ monoton artandır**: ya sabit kalır (plato), ya tırmanır. Asla aşağı inmez.

Odometre benzetmesi buraya da oturur: araba geri viteste gitse bile odometre geri saymaz.

> Azalabilen şey **$\dot y$**'dir (o anki ihlal şiddeti), $y$'nin kendisi değil.

**(3) $y$ için "ivme" tanımlanabilir mi?**

**Matematiksel olarak teknik olarak evet, ama anlamsız ve kullanılmıyor.**

Zincir kuralıyla yazılabilir:

$$\ddot y(t) = \frac{\partial q_c}{\partial x}\cdot \dot x(t) + \ldots$$

Ama pürüzsüz değildir:

| | $g<0$ (kısıt sağlanıyor) | $g>0$ (ihlal) | $g=0$'da |
|---|---|---|---|
| $q_c$ | 0 | $g^2$ | 0 — sürekli ✓ |
| $q_c'$ (→ $\dot y$) | 0 | $2g$ | 0 — sürekli ✓ |
| $q_c''$ (→ $\ddot y$) | 0 | $2$ | **0'dan 2'ye sıçrama ✗** |

$\dot y$ süreklidir (bu yüzden $C^1$-pürüzsüz — çözücünün ihtiyacı olan şey), ama $\ddot y$ kısıt sınırına tam değildiği anda **sıçrar**. Fiziksel bir ivme gibi düzgün davranmaz; bir anahtarın açılıp kapanması gibidir.

**Neden fiziksel ivmeyle aynı şey değil:**

- **Gerçek fizikte** ($v, a$): İvme, **dışarıdan gelen bir kuvvetin** sonucudur ($a = F/m$). Kuvvet bağımsız bir girdidir. Hız, bu bağımsız kuvvetin zaman içinde **birikmiş etkisidir**.
- **$y$'de böyle bir yapı yoktur.** $\dot y(t)$, o andaki $x(t), u(t)$'den **tamamen ve anlık olarak** belirlenir. Dışarıdan bağımsız bir "kuvvet" beslemez. $y$'nin kendine ait bir dinamiği, ataleti, hafızası yoktur — sadece o anki durumun bir **fonksiyonudur**.

$\ddot y$ hesaplamak **yeni bilgi vermez**; $\dot y$'nin değişim hızını gösterir, ki bu da zaten $\dot x, \dot u$'dan çıkarılabilir — dolaylı, gereksiz bir tekrar.

**Çerçeve neden $\ddot y$'yi hiç kullanmaz:** Tek ihtiyaç duyulan kısıt $y(t_f) - y(0) \le \epsilon_{LICQ}$'dur. Bu, sadece $y$'nin **kendisine** bakar. İhlal tek bir anda büyük sıçramayla da gelse, uzun süre küçük küçük birikse de — $y(t_f)$ aynıysa çerçeve için ikisi **eşdeğerdir**.

**Karşılaştırma tablosu:**

| Kavram | Fiziksel $v, a$ | $y, \dot y$ |
|---|---|---|
| "Hız" nereden gelir | Bağımsız kuvvet | Anlık $x(t), u(t)$'den doğrudan |
| İşaret değiştirebilir mi | Evet | **Hayır** — hep $\ge 0$ |
| "Konum" azalabilir mi | Evet | **Hayır** — monoton artan |
| "İvme" tanımlı mı | Evet, pürüzsüz | Teknik olarak var, sınırda sıçramalı, kullanılmıyor |
| Neden kullanılmıyor | — | Kısıt sadece $y(t_f)$'e bakar |

**(4) RK4 ≠ Newton metodu (sık yapılan karışıklık)**

| | Newton metodu | RK4 |
|---|---|---|
| Ne yapar | **Kök bulma** (root-finding) | **Diferansiyel denklem çözme** (ODE integration) |
| Sorusu | "Hangi $x$ değeri $f(x)=0$ yapar?" | "$\dot x = f(x,u)$ ve $x(t)$ biliniyorsa, $x(t+\delta t)$ nedir?" |
| Mekanizma | Tahmin → hata ölç → düzelt → tekrar | Zamanı adım adım ileri sar |
| Analoji | Hedefi ıskalayarak yaklaşan okçu | Hız göstergesine bakıp yol hesaplayan navigasyon |

`rk4_steps_dyn = 20` demek: her düğüm aralığını 20 küçük **zaman adımına** bölmek. Bu bir kök bulma iterasyonu değil, **zaman adımı inceltme**dir.

---

## 6. Kod deposu analizi

Kaynak: `https://github.com/sametuzun781/CT-cSTC` (asıl depo — §1.2'deki uyarıya bakınız)

### 6.1 Yapı

Asıl kod **tek bir Jupyter notebook**'tur: `CT-cSTC.ipynb`, 55 hücre, Python 3.10.13, kernel adı `swarm`. Notebook tarihi **20 Ocak 2026** — makaleden sonra güncellenmiş.

| Hücre | Bölüm | İçerik | Makale karşılığı |
|---|---|---|---|
| 6 | Kütüphaneler | Bağımlılıklar, matplotlib ayarları | — |
| 8–9 | **D-GMSR** | `gmsr_and`, `gmsr_or`, `UNTIL` | III.A |
| 11 | Gif maker | Animasyon üretimi | — |
| 13 | Print | Iterasyon logu yardımcıları | — |
| 16–17 | **Entegrasyon** | RK4, NumPy ve JAX versiyonları | III.B.3 |
| 19–20 | **Discretization** | `dVdt` — STM türetimi | III.B.3 |
| 22 | Nonlineer maliyet | SCP yakınsama kontrolü | III.B.5 |
| 24 | **Konveks alt-problem** | CVXPY modeli | Eq. (11) |
| 26 | **Prox-linear** | 147 satır — ana SCP döngüsü + Algorithm 1 | III.B.5 |
| 28 | RUN | Çözümün yeniden entegrasyonu | — |
| 31 | Utils | `euler_to_quat`, `CBI_fcn`, `skew`, `omega` | II.A |
| 33 | **Parametreler** | Tablo 3 + makalede olmayan ~20 parametre | Tablo 3 |
| 35 | Başlangıç tahmini | `initialize_trajectory` | IV |
| 37 | **Ölçekleme** | `rl_scale_params` | IV ("affine scaling") |
| 39 | **Dinamik** | 119 satır — dinamik + tüm kısıt/STC cezaları | II.A, II.B, II.D |
| 41 | **Maliyet & kısıtlar** | `rl_cost_fcn`, `rl_cons_fcn` | II.B, II.C, Eq. (7a) |
| 43–45 | Grafik | 823 satır — Fig. 1 ve 2'yi üreten kod | Fig. 1–2 |
| 47–53 | Çalıştırma | Parametre → jit → prox-linear → plot | IV |

### 6.2 Teknoloji yığını

```python
import numpy as np
import cvxpy as cp
import jax
import jax.numpy as jnp
from jax import jit, jacfwd, vmap, lax, config, Array
config.update("jax_enable_x64", True)
```

| Bileşen | Rol |
|---|---|
| **JAX** | Otomatik türev (`jacfwd`) + JIT derleme (`jit`) |
| **CVXPY** | Konveks alt-problemin modellenmesi |
| **MOSEK / CLARABEL** | SOCP çözücüsü |
| NumPy, Matplotlib, PIL | Sayısal işlem, grafik, animasyon |

> `config.update("jax_enable_x64", True)` — çift hassasiyet (float64) zorlanıyor. JAX varsayılanı float32'dir; optimizasyon için yetersiz olur.

### 6.3 Jacobian'lar otomatik türev ile

```python
params['f_func'] = jit(dynamics)
params['A_func'] = jit(jacfwd(dynamics, argnums=0))   # ∂f/∂x
params['B_func'] = jit(jacfwd(dynamics, argnums=1))   # ∂f/∂u
```

`jacfwd` = **forward-mode automatic differentiation** (ileri modlu otomatik türev). Elle veya sembolik türev **yoktur**.

> Makale $A(\tau) = \partial f/\partial \tilde x$ der ama nasıl hesaplandığını söylemez. Kod cevabı verir.
>
> **Tez için not:** Bu, PROJE_BAGLAMI §6'daki *"Jacobian türetimi: MATLAB Symbolic Toolbox `jacobian()` önerildi"* notuna doğrudan alternatiftir. Otomatik türev genelde daha hızlıdır ve **ifade şişmesi (expression swell)** sorunu yaşamaz. MATLAB karşılıkları: `dlgradient` (Deep Learning Toolbox) veya **CasADi**.

### 6.4 Durum vektörü — 15 boyut, 14'ü fiziksel

```
x[0]      → kütle m                      (1)
x[1:4]    → konum r_I                    (3)
x[4:7]    → hız v_I                      (3)
x[7:11]   → quaternion q_{B←I}           (4)
x[11:14]  → açısal hız ω_B               (3)
x[14]     → y  ← CTCS ihlal sayacı       (1)  ← FİZİKSEL DEĞİL
                                        ----
                                   n_x =  15
```

Kontrol vektörü, `n_u = 5`:

```
u[0] → T       itki büyüklüğü
u[1] → δ^e     motor gimbal sapma açısı
u[2] → φ^e     motor gimbal azimut açısı
u[3] → δ^b     boresight sapma açısı
u[4] → φ^b     boresight azimut açısı
```

Ayrıca time-dilation değişkeni $s$ ayrı bir dizi olarak (`sigma`) taşınır.

### 6.5 Parametreler

#### 6.5.1 Makalenin Tablo 3'üyle eşleşenler

| Parametre | Kod değeri | Makale |
|---|---|---|
| $K$ | 15 | 15 ✓ |
| $g_0$ | 9.806 m/s² | 9.806 ✓ |
| $I_{sp}$ | 330 s | 330 ✓ |
| $\rho$ | 1.225 kg/m³ | 1.225 ✓ |
| $S_A$ | 545 m² | 545 ✓ |
| $C_A$ | diag([0.4068, 0.4068, 0.0522]) | ✓ |
| $m_i / m_{dry}$ | 100 000 / 85 000 kg | ✓ |
| $r_i$ | (200, 200, 500) m | ✓ |
| $v_i / v_f$ | (0,0,−50) / (0,0,−5) m/s | ✓ |
| $r_{cm,B}$ | (0, 0, −14) m | ✓ |
| $r_{cp,B}$ | (0, 0, 3) m | ✓ |
| $J_B$ | diag([60, 60, 1.5]), dinamikte $\times m(t)$ | ✓ |
| $\omega_{max}$ | 90 °/s | ✓ |
| $\theta_{max}$ | 90° | ✓ |
| $\delta^e_{max}$ | 10° | ✓ |
| $\delta^b_{max}$ | 20° | ✓ |
| $h^{trig}_1 / h^{trig}_2$ | 100 / 200 m | ✓ |
| $v^{trig}_I$ | 35 m/s | ✓ |
| $\theta^{trig}$ | 60° | ✓ |
| $v^{stc}_I$ | 20 m/s | ✓ |
| $\omega^{stc}$ | 2.5 °/s | ✓ |
| $\theta^{stc}$ | 5° | ✓ |
| $\psi^{stc}$ | 5° | ✓ |
| $\delta^{stc}$ | 1° | ✓ |
| Başlangıç $t_f$ | 21 s | ✓ |

**İtki limitleri** (kodda çarpanlı yazılmış, sonuçlar Tablo 3 ile eşleşir):

| Kod | Hesap | Makale karşılığı |
|---|---|---|
| `T_max = 2200000 * 3.0` | 6 600 000 N | $T^{stc_2}_{max}$ ✓ |
| `T_min = 2200000 * 0.4 * 3.0` | 2 640 000 N | $T^{stc_2}_{min}$ ✓ |
| `T_max_aft = 2200000` | 2 200 000 N | $T^{stc_1}_{max}$ ✓ |
| `T_min_aft = 2200000 * 0.4` | 880 000 N | $T^{stc_1}_{min}$ ✓ |

> `_aft` eki "after trigger" (tetikleme sonrası) anlamındadır — hız ve tilt eşiklerinin altına inildikten sonra geçerli olan dar itki bandı.

#### 6.5.2 Makalede hiç geçmeyen parametreler

| Parametre | Değer | Ne işe yarar |
|---|---|---|
| `w_stc_1` | 100 | STC ceza ağırlığı — grup 1 |
| `w_stc_2` | 1000 | STC ceza ağırlığı — grup 2 |
| `w_stc_3` | 600 | STC ceza ağırlığı — grup 3 |
| `w_sigma` | 150 | Zaman maliyeti ağırlığı |
| `w_con_dyn` | 1e4 | Dinamik ihlali cezası ($w^{dyn}_{eq}$) |
| `w_ptr` | $2^{12}$ = 4096 | Prox-linear güven bölgesi başlangıç ağırlığı |
| `w_ptr_min` | 0.001 | Minimum güven bölgesi ağırlığı |
| `w_ds` | 20 | Zaman için ekstra ceza |
| `ite` | 85 | Maksimum iterasyon |
| `ptr_term` | 1e-4 | Sonlanma koşulu |
| `adaptive_step` | False | Adaptif ağırlık güncelleme (ilk çalıştırmada kapalı) |
| `r0, r1, r2` | 0.01, 0.1, 0.8 | Algorithm 1'in $\beta_1, \beta_2$ eşikleri |
| `beta_ctcs` | 1e-4 | $\epsilon_{LICQ}$ |
| `alpha_*` | 0.92–1.10 | $\delta_{LICQ}$ kısıt sıkılaştırma çarpanları (§5.4.6) |
| `rk4_steps_dyn` | 20 | Dinamik entegrasyonu alt-adım sayısı |
| `rk4_steps_J` | 20 | Jacobian entegrasyonu alt-adım sayısı |
| `N_dt` | 10 | — |
| `min_sigma / max_sigma` | $0.5\,t_{scp}$ / $10\,t_{scp}$ | Time-dilation faktörü sınırları |
| `l_r / l_h` | 4.5 m / 50 m | Gövde yarıçapı ve yüksekliği — **yalnızca çizim için**, dinamiğe girmez |
| `inp_param` | 'FOH' | First-Order Hold kontrol parametrizasyonu |
| `free_final_time` | True | Serbest bitiş zamanı bayrağı |
| `time_dil` | True | Time-dilation bayrağı |

> ⚠️ Son iki bayrak önemlidir: **kapatılabilirler.** Bu, sabit-$t_f$ ile karşılaştırma yapmayı kolaylaştırır.

### 6.6 Amaç fonksiyonu — gerçek hali

Makale Eq. (7a): $\min\; \tfrac{1}{2}(\sum s_k + \sum s_k)$ — düz $t_f$.

Kod (`rl_cost_fcn`):

```python
sigma_cost = params['w_sigma'] * cp.pnorm(sigma[0, :], 1) / params['t_f']
vehicle_cost += sigma_cost
```

Yani **ağırlıklı ve normalize edilmiş** ($w_\sigma = 150$, başlangıç tahmini $t_f = 21$'e bölünmüş). Matematiksel olarak eşdeğerdir (pozitif skalerle çarpım optimumu değiştirmez), ama sayısal ölçekleme için yapılmıştır.

> ✅ **Case study için iyi haber:** Maliyet fonksiyonu 30 satırlık, tek amaçlı ve içinde zaten bir ağırlık var. Yakıt terimi eklemek tek satır:
>
> ```python
> fuel_cost = w_fuel * (-X[0, -1]) / params['m_wet']   # son düğümdeki kütle
> vehicle_cost += fuel_cost
> ```

### 6.7 Konveks kısıtlar (`rl_cons_fcn`)

```python
# Time-dilation sınırları
sigma[0,:] >= min_sigma ;  sigma[0,:] <= max_sigma

# Sınır koşulları
X[:, 0] == x_init
X[1:-1, -1] == x_final[1:-1]          # kütle ve y hariç

# CTCS
X[-1:, 1:] - X[-1:, 0:-1] <= beta_ctcs

# Durum kısıtı
X[0, :] >= m_dry                       # minimum kütle

# Kontrol kısıtları
U[0,:] <= T_max ;  T_min_aft <= U[0,:]
-δ^e_max <= U[1,:] <= δ^e_max
-δ^b_max <= U[3,:] <= δ^b_max
```

> Dikkat: `X[1:-1, -1] == x_final[1:-1]` — son düğümde kütle (`X[0]`) ve $y$ (`X[-1]`) sabitlenmez. Kütle serbesttir (yakıt tüketimi optimizasyonun sonucu), $y$ ise CTCS kısıtıyla ayrıca kontrol edilir.

### 6.8 Ölçekleme

```python
r_scale = np.linalg.norm(r_I_init),   # √(200² + 200² + 500²) ≈ 574.5 m
m_scale = m_wet,                      # 100 000 kg
```

`rl_scale_params` (hücre 37) tüm değişkenleri bunlara böler.

> ⚠️ **Amaç fonksiyonunu değiştirirken bu bölüm gözden geçirilmelidir.** $t_f \sim 20$ ile $m \sim 10^5$ arasında **4 mertebe** fark vardır; ölçekleme güncellenmezse sayısal performans bozulur ve bu "algoritma çalışmıyor" gibi görünen ama aslında ölçekleme kaynaklı bir hataya dönüşür.

### 6.9 Başlangıç tahmini

```python
X_last = np.linspace(x_init, x_final, K).T          # doğrusal interpolasyon
U_last = np.zeros((n_u, K))
U_last[0, :] = (T_max + T_min) / 2                  # sabit orta itki
sigma_last = t_scp * np.ones((1, K - 1))            # eşit zaman dilimleri
```

Gimbal açıları sıfır. Makalenin Bölüm IV'teki tarifiyle birebir uyumlu.

### 6.10 Zamanlama verisi — makalede yok, notebook'ta var

Notebook'un kayıtlı çıktısında iterasyon logu durur:

| Metrik | Değer |
|---|---|
| İterasyon başına toplam (T-Ite) | 0.08 – 0.14 s |
| Discretization / STM entegrasyonu (T-Disc) | ~0.01 s |
| Konveks alt-problem / çözücü (T-SubP) | 0.08 – 0.13 s |

> **Zamanın ~%90'ı çözücüdedir**, JAX tarafı neredeyse bedava. Makalenin PIPG'yi "verimli implementasyonlar için" önermesinin sebebi tam olarak budur.
>
> ⚠️ **İki uyarı:** (a) donanım bilgisi yoktur, (b) çıktıda JAX'in **optimizasyonsuz derlendiğine** dair bir XLA uyarısı vardır (`XLA was built without compiler optimizations`). Bu sayılar tezde alıntılanacak referans değerler **değildir** — kendi benchmark'ınızı yapma gerekliliğini destekleyen ipuçlarıdır.

### 6.11 Yakınsama notu

Kayıtlı çalıştırmanın sonunda:

```
Maximum number of iterations reached without convergence.
```

Bağlam: Bu **ikinci (warm-start)** çalıştırmadır ve iterasyon limiti 15'e düşürülmüştür (`rl_params_sc['ite'] = 15`, `adaptive_step = True`). Dinamik ihlali (`dyn_cost`) 0.05'e inmiştir — pratikte çözüm oturmuş, ama resmi sonlanma kriterine ($10^{-4}$) ulaşmadan limit dolmuştur.

> Panik gerektirmez, ama "kodu çalıştırınca temiz yakınsama göreceğiz" beklentisiyle girilmemelidir.

### 6.12 Warm-start yapısı

```python
prox_results = prox_linear(rl_params_sc)          # 1. çözüm (soğuk başlangıç, ite=85)
rl_results_sc = RUN(...)

# Warm start
rl_params_sc['ite'] = 15
rl_params_sc['adaptive_step'] = True
rl_params_sc['X_last'] = rl_results_sc['x_nmpc_all'][0, :, :].T
rl_params_sc['U_last'] = rl_results_sc['u_nmpc_all'][0, :, :].T
rl_params_sc['sigma_last'] = rl_results_sc['sigma_nmpc_all']

prox_results = prox_linear(rl_params_sc)          # 2. çözüm (sıcak başlangıç)
```

> Değişken adlarındaki `_nmpc_` eki dikkat çekicidir — MPC yapısını çağrıştırır, ancak **kayan ufuk (receding horizon) döngüsü uygulanmamıştır.** Bu, açık-döngü yörünge üretimidir.

### 6.13 Kodda olmayanlar

| Eksik | Sonuç |
|---|---|
| `requirements.txt` | Bağımlılıklar elle kurulacak (JAX kurulumu zahmetli olabilir) |
| `.py` modülü | Her şey tek notebook — modülerleştirme gerekecek |
| **MPC / kayan ufuk döngüsü** | Açık-döngü yörünge üretimi. MPC için [62] |
| MATLAB / Simulink | Saf Python |
| Test / doğrulama scripti | Sonuçların doğrulanması bize kalıyor |
| **$r_{CG}(t)$ kayması** | `r_cm = [0,0,-14]` **sabit**. Atalet sadece kütleyle ölçekleniyor: `J_B_m = J_B_pre * x[0]` |
| Sloshing | Yok — bizim §4.2 kararımızla uyumlu ✓ |
| Özel çözücü (PIPG) | Yok (§5.3) |

### 6.14 `UNTIL` fonksiyonu

Makalenin sonuç bölümü *"eventually ve until için sürekli-zaman formülasyonu gelecek çalışma"* der. Ancak kodda hücre 9'da **`UNTIL` fonksiyonu vardır** (35 satır). Notebook makaleden sonra (20 Ocak 2026) güncellenmiş olduğu için, muhtemelen devam çalışmasının izidir.

---

## 7. Tespitler ve errata

Bu bölüm, makale ile kod arasındaki tutarsızlıkları ve makaledeki hataları toplar. PROJE_BAGLAMI §4.1'e taşınmalıdır.

### 7.1 ⚠️ Tablo 3'te quaternion yazım hatası

Makale, başlangıç quaternion'ı için şunu yazar:

$$q_{B\leftarrow I_i} = (\sqrt2,\; \sqrt2,\; 0,\; 0)$$

Bu quaternion'ın normu **2**'dir — **geçersizdir.** Birim quaternion olmak zorundadır.

Kod `euler_to_quat([90°, 0, 0])` çağırır; sayısal olarak doğrulandı:

$$q = \left(\tfrac{\sqrt2}{2},\; \tfrac{\sqrt2}{2},\; 0,\; 0\right) = (0.7071,\; 0.7071,\; 0,\; 0), \qquad \|q\| = 1 \;\checkmark$$

> **Doğrusu $\sqrt2/2$'dir; makale bölümü düşürmüştür.** Wang & Song'daki Eq.(4) hatasıyla aynı sınıfta bir bulgu.
>
> **Fiziksel anlamı:** Roket, x ekseni etrafında **90° yatık** başlar. Sonda dik ($q_f = (1,0,0,0)$) biter. Yani manevra "yatay uçuştan dikey inişe geçiş"tir.

### 7.2 ⚠️ Glideslope açı konvansiyonu uyuşmazlığı

Makalenin Bölüm II.B'deki denklemi:

$$\tan(\gamma_{max}) \left\| [e_1\; e_2]^\top r_I(t) \right\|_2 \le e_3^\top r_I(t)$$

Tablo 3: $\gamma_{max} = 35°$.

Kod ise **tümleyen açıyı** kullanır:

```python
gs_max = 90 - 35,            # = 55
...
gs_cons_pre = np.tan(np.deg2rad(params['gs_max'])) * (x[1]**2 + x[2]**2)**0.5 - x[3]
```

Sayısal fark:

| | Kısıt | Düşeyden izin verilen açı |
|---|---|---|
| Makale denklemi ($\gamma_{max}=35$ konularak) | $\tan(35°)\cdot\text{yatay} \le \text{irtifa}$ | $\arctan(1/\tan 35°) = 55°$ |
| Kod ($\gamma_{max}=55$ konularak) | $\tan(55°)\cdot\text{yatay} \le \text{irtifa}$ | $\arctan(1/\tan 55°) = 35°$ |

$\tan(55°) = \cot(35°) = 1.428$ — yani ikisi birbirinin tersidir.

**Yorum:** Makalenin Fig. 2'sinde glideslope açısı $\gamma$ ~32°'den 0'a düşer ve üst sınır 35°'de çizilidir — bu, $\gamma$'nın **düşeyden** ölçüldüğünü gösterir. Kod bu konvansiyonu uygular. Ancak Bölüm II.B'deki denklem, $\gamma_{max}$'ı **yataydan** ölçülüyormuş gibi yazar. Denklem ve Tablo 3 **farklı konvansiyon** kullanır.

> ⚠️ **Tez için kritik:** Kodu kendi modelinize uyarlarken bu konvansiyonu mutlaka netleştirin. Yanlış tarafı seçerseniz, kısıt olması gerekenden çok daha gevşek (veya sıkı) olur ve bu sessizce yanlış sonuç üretir. **Kodun versiyonu gerçekten çalıştırılmış olandır.**
>
> Aynı durum STC glideslope'u için de geçerlidir: kod `gs_stc_cons = 90 - 5`, makale $\gamma^{stc} = 5°$.

### 7.3 Makale LCvx'i ne kanıtlar ne kullanır

LCvx yalnızca Introduction'da literatür arka planı olarak anılır. Kanıt [12], [13], [15]–[19]'a havale edilir. Makalenin kendisi **SCP** tabanlıdır.

> §7 (PROJE_BAGLAMI) "Liu (2017) — lossless convexification kanıtı" boşluğu **hâlâ açıktır.** Açıkmeşe & Ploen (2007) [12] okunmalıdır.

### 7.4 Hesaplama süresi makalede verilmez

Makale kendini "real-time implementable" olarak tanımlar ama Bölüm IV'te **hiçbir hesaplama süresi yoktur** — ne çözücü süresi, ne iterasyon sayısı, ne donanım bilgisi. Yalnızca notebook'un kayıtlı çıktısında dolaylı veri vardır (§6.10).

> Bu, PROJE_BAGLAMI §7'deki *"Hesaplama süresi ölçümü — teorik tahmin değil, gerçek benchmark"* maddesini doğrudan destekler: literatürdeki bu boşluk, tezin bir katkı alanı olabilir.

### 7.5 $\delta_{LICQ}$ değerleri yalnızca kodda

Makale *"her $\epsilon_{LICQ}$ için karşılık gelen bir $\delta_{LICQ}$ vardır"* der, sayı vermez. Kod somut değerleri verir (§5.4.6 tablosu).

### 7.6 Amaç fonksiyonu kodda ağırlıklı/normalize

Makale düz $t_f$ yazar; kod $w_\sigma \|s\|_1 / t_f$ kullanır (§6.6). Matematiksel olarak eşdeğer, sayısal olarak farklı davranır.

### 7.7 STC ceza ağırlıkları makalede yok

`w_stc_1/2/3 = 100/1000/600`. Bu değerler ayar (tuning) parametreleridir ve makalede hiç geçmez. Farklı senaryolarda yeniden ayarlanmaları gerekebilir.

### 7.8 Model basitleştirmesi: $r_{cm}$ sabit

Kod `r_cm = [0,0,-14]` sabit alır; atalet yalnızca kütleyle ölçeklenir. Bizim §4.2 kararımız (CG kayması + $\mathbf I(t)$ güncellemesi) **daha ayrıntılıdır**. Sorun değil — bizimki daha gerçekçi — ama kodu doğrudan çalıştırırken bu fark bilinerek yapılmalıdır.

---

## 8. Tez entegrasyonu

### 8.1 Artımlı uygulama yolu

Çerçeve katmanlıdır ve her katman ayrı bir bayrak/parametre ile kontrol edilir. Bu, PROJE_BAGLAMI §4.2'deki *"önce temel dikey iniş başarısı, sonra yapısal yük / açısal hız / gözlem alanı kısıtları"* kararıyla birebir örtüşür.

| Aşama | Ne yapılır | Kod tarafında |
|---|---|---|
| **1** | STC'leri kapat, düz 6-DoF dikey iniş | `f.at[14]`'teki STC terimlerini çıkar, sadece `tilt_ang_cons + ang_vel_cons + gs_cons` kalsın |
| **2** | Temel konveks kısıtları ekle | Zaten ayrı satırlarda duruyor |
| **3** | Angara 1.2 parametrelerini koy | `rl_params_fcn()` içindeki sayılar |
| **4** | Amaç fonksiyonu case study | `rl_cost_fcn` — tek fonksiyon |
| **5** | STC'leri kademeli aç | D-GMSR terimlerini geri ekle |
| **6** | MPC katmanı | [62] + kayan ufuk |
| **7** | Simulink entegrasyonu | Aşağıdaki üç yol |

> ✅ **Kodun yapısı bu artımlı yolu zaten destekler** — STC terimleri dinamik fonksiyonunda **ayrı ayrı satırlarda** durur ve tek tek kapatılabilirler.

### 8.2 Amaç fonksiyonu case study planı

Time-dilation yapısı sayesinde **her iki amaç fonksiyonu da lineerdir** — bu, karşılaştırmayı teknik olarak bedava kılar.

**Min zaman** (makalenin tercihi, Eq. 7a):
$$J_{time} = \tilde t_E \tilde x(1) = \tfrac{1}{2}\Big(\sum_{k=1}^{K-1} {}^sE\tilde u_k + \sum_{k=2}^{K} {}^sE\tilde u_k\Big)$$
$s$ zaten kontrol değişkenidir ve FOH ile parametrize edilmiştir → toplam süre $s$ değerlerinin trapez toplamıdır. **Karar değişkenlerinde lineer.**

**Min yakıt:**
$$J_{fuel} = -\,{}^mE\,\tilde x_K$$
Son düğümdeki kütle elemanının negatifi. $\dot m = -\alpha_{\dot m}\|T_B\|$ olduğu için kütle zaten yakıt tüketiminin integralini taşır — ayrıca integral almaya gerek yoktur. **Lineer.**

**Case matrisi:**

| Case | Amaç fonksiyonu | Cevapladığı soru |
|---|---|---|
| **A** | $\min\; t_f$ | Makalenin baseline'ı — ne kadar hızlı inilebilir? |
| **B** | $\min\; (m_i - m_f)$ | Wang & Song'un kriteri — ne kadar az yakıtla? |
| **C** | $\min\; w_1 t_f + w_2 (m_i - m_f)$ | Ağırlık süpürmesi → **Pareto eğrisi** |
| **D** | $\min\; \|r(t_f) - r_f\|$ | Minimum sapma (opsiyonel) |

Case C'deki $w_1/w_2$ taraması, **süre–yakıt Pareto eğrisi** verir; tez metninde tek başına bir şekil olarak durur ve "neden bu kriteri seçtim" sorusunun görsel cevabıdır.

**Her case için kaydedilecek metrikler:**

- $t_f$ (s) ve tüketilen yakıt (kg)
- İtki profili $T(t)$ — doygunluğa ne kadar giriyor
- Maksimum tilt açısı, maksimum açısal hız
- Gimbal açısı geçmişi (aktüatör yükü)
- SCP iterasyon sayısı ve **çözüm süresi**

**Fizik olarak beklenen (tahmin — ölçülecek):**

- **Min zaman** itkiyi büyük ölçüde üst sınırda tutmaya eğilimlidir. Hızlı düşüş, sonda sert fren. Yakıt tüketimi yüksek.
- **Min yakıt** ilk bakışta "yavaş in, az yak" der gibi görünür ama fizik tersini söyler: **yerçekimi kaybı (gravity loss)**. Havada asılı durulan her saniye, sadece yerçekimini dengelemek için yakıt yakılır — hiçbir ilerleme kaydetmeden. Bu yüzden min-yakıt çözümü de süresiz uzamaz; kendini sınırlar. Klasik sonuç genellikle bang-bang benzeri bir profildir.

> İki case'in **beklenenden yakın** $t_f$ değerleri vermesi şaşırtıcı olmaz. Öyle çıkarsa, tezde açıklanmaya değer bir bulgudur.

**Üç pratik tuzak:**

1. **Serbest $t_f$ şarttır.** $t_f$ sabitlenirse karşılaştırma anlamsızlaşır. Time-dilation zaten bunu sağlar (`free_final_time = True`).
2. **Ölçekleme yeniden ayarlanmalıdır** (§6.8).
3. **Adil karşılaştırma:** aynı başlangıç tahmini, aynı kısıt seti, aynı $K$, aynı tolerans. Tek değişen amaç fonksiyonu olmalıdır.

### 8.3 Wang & Song ile karşılaştırma

> ⭐ **Tez metninde tartışılmaya değer bir karşılaştırma noktası: iki farklı ekip, aynı fiziksel probleme iki farklı optimallik kriteriyle yaklaşmıştır.**

| | [[Wang_Song_2018_KAPSAMLI_NOT|Wang & Song (2018)]] | Uzun ve ark. (2025) |
|---|---|---|
| Model | 3-DoF nokta kütle | **6-DoF** (quaternion + gimbal) |
| Yöntem | Convex MPC (SOC relaxation + successive linearization) | **SCP** (CT-SCvx + D-GMSR) |
| Amaç fonksiyonu | $J = -z_f$ (**yakıt maksimizasyonu**) | $\min t_f$ (**minimum zaman**) |
| Serbest $t_f$ | Evet (katkı 1) | Evet (time-dilation) |
| MPC | Evet (novel receding horizon) | **Hayır** (açık-döngü) |
| Sürekli-zaman garantisi | Hayır | **Evet** |
| Mantıksal kısıtlar | Hayır | **Evet** (D-GMSR) |
| Aerodinamik | Var | Var |
| Çözücü | MOSEK | MOSEK / CLARABEL |

> Bu fark, PROJE_BAGLAMI §4.3'teki **açık tasarım kararına** — *"alt katman hangi maliyet fonksiyonunu kullanacak?"* — doğrudan bir **üçüncü seçenek** sunar.

### 8.4 Simulink entegrasyon seçenekleri

| Yol | Açıklama | Avantaj / dezavantaj |
|---|---|---|
| **(a)** MATLAB'da yeniden yaz | YALMIP/MOSEK ile | Tez için en temiz, en çok iş |
| **(b)** Simulink'ten Python çağır | MATLAB `pyrunfile` | Orta zorluk, bağımlılık riski |
| **(c)** Çevrimdışı yörünge üret, Simulink'te takip et | Yörünge dosyası + takip kontrolcüsü | **En hızlı ilk milestone** |

> Önerilen sıra: **(c) ile başla** (görünür sonuç hızlı gelir, Unity'ye de hemen akar), sonra **(a)**'ya geç.

### 8.5 Mimari yerleşim

Bu SCP çözücüsü, PROJE_BAGLAMI §4.3'teki hiyerarşik mimaride **üst katman** (yavaş, yörünge planlayıcı) olarak oturur:

```
ÜST KATMAN (yavaş, ~1-2s)          ALT KATMAN (hızlı, ~0.1s)
CT-SCvx / SCP yörünge planlama  →  Anlık tutum/güdüm komutu
J = min t_f  veya  min yakıt       J = ? ← HÂLÂ AÇIK
```

---

## 9. Açık sorular ve yapılacaklar

### 9.1 Bu makaleden doğan açık sorular

- [ ] Glideslope konvansiyonu (§7.2) — hangi tarafın doğru olduğu kendi modelimizde netleştirilmeli
- [ ] $w_{stc}$ ağırlıkları farklı senaryolarda nasıl ayarlanır? Makale yol göstermiyor
- [ ] `rk4_steps_dyn = 20` yeterli mi? Daha az alt-adımla sürekli-zaman garantisi ne kadar bozulur?
- [ ] Amaç fonksiyonu değiştiğinde ölçekleme nasıl güncellenmeli?
- [ ] Prox-linear yakınsama kriterine gerçekten ulaşılıyor mu (§6.11)?

### 9.2 Analiz sırası — sonraki adımlar

- [ ] **Bölüm II.A — Rocket Dynamics** (kodun `rl_dynamics` fonksiyonuyla birebir eşleşir)
- [ ] Bölüm II.B, II.C — kısıtlar ve sınır koşulları
- [ ] Bölüm II.D — compound STC'ler
- [ ] Bölüm III.A — D-GMSR parametrizasyonu
- [ ] Bölüm III.B — CT-SCvx (5 alt bölüm)
- [ ] Bölüm IV — sayısal sonuçlar
- [ ] Bölüm V — sonuç

### 9.3 Okunacak ek kaynaklar (öncelik sırasıyla)

| Öncelik | Kaynak | Neden |
|---|---|---|
| ⭐⭐⭐ | **[62]** Uzun ve ark. (2024), NMPC with CT-SCvx | **MPC katmanı** — tezin çekirdeği |
| ⭐⭐⭐ | **[55]** Elango ve ark. (2024), CT-SCvx ana makalesi | Bu makalenin dayandığı çerçeve; $\epsilon_{LICQ}/\delta_{LICQ}$ teorisi burada |
| ⭐⭐ | [75] Uzun ve ark. (2024), D-GMSR | Soundness/completeness kanıtları |
| ⭐⭐ | [37] Szmuk, Reynolds & Açıkmeşe (2020), STC | Dinamik ve kısıt modeli buradan alınmış |
| ⭐⭐ | [54] Kamath ve ark. (2023), dual quaternion 6-DoF | Gimbal motor modeli buradan alınmış |
| ⭐ | [12] Açıkmeşe & Ploen (2007) | LCvx kanıtı — §7.3'teki açık boşluk |
| ⭐ | [35] Szmuk & Açıkmeşe (2018), free-final-time | Time-dilation kökeni |
| ⭐ | [33] Drusvyatskiy & Paquette (2019) | Prox-linear yakınsama teorisi |

---

## 10. Terimler sözlüğü

| Kısaltma | Açılım | Türkçe |
|---|---|---|
| **PDG** | Powered Descent Guidance | Güdümlü motor inişi rehberliği |
| **DoF** | Degrees of Freedom | Serbestlik derecesi |
| **VTVL** | Vertical-Takeoff/Vertical-Landing | Dikey kalkış / dikey iniş |
| **IPM** | Interior Point Method | İç nokta yöntemi |
| **LCvx** | Lossless Convexification | Kayıpsız konvekşleştirme |
| **PMP** | Pontryagin's Maximum Principle | Pontryagin maksimum prensibi |
| **SOCP** | Second-Order Cone Programming | İkinci dereceden koni programlama |
| **SOC** | Second-Order Cone | İkinci dereceden koni |
| **G-FOLD** | Guidance for Fuel Optimal Large Divert | Yakıt-optimal büyük sapma rehberliği |
| **SCP** | Sequential Convex Programming | Ardışık konveks programlama |
| **SCvx** | Successive Convexification | Ardışık konvekşleştirme |
| **CT-SCvx** | Continuous-Time Successive Convexification | Sürekli-zaman ardışık konvekşleştirme |
| **CTCS** | Continuous-Time Constraint Satisfaction | Sürekli-zaman kısıt sağlama |
| **STC** | State-Triggered Constraint | Durum-tetiklemeli kısıt |
| **cSTC** | compound State-Triggered Constraint | Bileşik durum-tetiklemeli kısıt |
| **STL** | Signal Temporal Logic | Sinyal zamansal mantığı |
| **D-GMSR** | Discrete Generalized Mean-based Smooth Robustness | Ayrık genelleştirilmiş ortalama tabanlı pürüzsüz robustluk |
| **MIP** | Mixed-Integer Problem | Karma tamsayılı problem |
| **PIPG** | Proportional-Integral Projected Gradient | Oransal-integral izdüşümlü gradyan |
| **NMPC** | Nonlinear Model Predictive Control | Doğrusal olmayan model öngörülü kontrol |
| **MPC** | Model Predictive Control | Model öngörülü kontrol |
| **SPLICE** | Safe and Precise Landing – Integrated Capabilities Evolution | NASA hassas iniş programı |
| **MSL** | Mars Science Laboratory | Curiosity görevi |
| **LICQ** | Linear Independence Constraint Qualification | Lineer bağımsızlık kısıt niteliği |
| **KKT** | Karush-Kuhn-Tucker | Optimallik koşulları |
| **STM** | State Transition Matrix | Durum geçiş matrisi |
| **FOH** | First-Order Hold | Birinci mertebe tutucu |
| **RK4** | Runge-Kutta 4th order | 4. mertebe Runge-Kutta |
| **DCM** | Direction Cosine Matrix | Yön kosinüs matrisi |
| **CG / CM** | Center of Gravity / Center of Mass | Ağırlık / kütle merkezi |
| **CP** | Center of Pressure | Basınç merkezi |
| **LOS** | Line of Sight | Görüş hattı |
| **a.e.** | almost everywhere | hemen hemen her yerde |

### 10.1 Sembol sözlüğü (Introduction kapsamında)

| Sembol | Anlam | Birim |
|---|---|---|
| $T$ | İtki büyüklüğü | N |
| $\Gamma$ | LCvx slack değişkeni (itki üst zarfı) | N |
| $T_{min}, T_{max}$ | İtki alt/üst sınırları | N |
| $t_f$ | Bitiş zamanı (serbest) | s |
| $y$ | **CTCS ihlal sayacı (augmented state)** | — |
| $\dot y$ | Anlık toplam ihlal şiddeti | 1/s |
| $q_c(\cdot)$ | Dış ceza fonksiyonu, $\max(0,\cdot)^2$ | — |
| $\epsilon_{LICQ}$ | CTCS gevşetme toleransı (kod: 1e-4) | — |
| $\delta_{LICQ}$ | Kısıt sıkılaştırma payı (kod: `alpha_*`) | — |
| $s(\tau)$ | Time-dilation faktörü | s |
| $\tau$ | Normalize zaman, $\tau \in [0,1]$ | — |
| $K$ | Düğüm sayısı | — |
| $\lambda(t)$ | Costate (eş-durum) vektörü | — |

---

## 11. Referans haritası

Introduction'da atıf verilen kaynakların tematik gruplaması — hangi konu için hangisine bakılmalı:

| Konu | Kaynaklar |
|---|---|
| **Polinom rehberlik / tarihçe** | [2–7] (Apollo, MSL, Mars 2020) |
| **Konveks optimizasyon temeli** | [9–11] (IPM, Boyd & Vandenberghe) |
| **LCvx çekirdeği** | **[12]** (Açıkmeşe & Ploen 2007 — ana kanıt) |
| **LCvx genişletmeleri** | [13] genel kontrol kümeleri, [14] min-error, [15] pointing, **[16] afin durum kısıtları**, **[17] kuadratik durum kısıtları**, [18] nonlineer dinamik, [19] binary |
| **Özel SOCP çözücüleri** | [20, 21] |
| **G-FOLD / uçuş testi** | [22–26] (Xombie) |
| **SCP tutorial / genel** | **[27, 28]** (Malyuta ve ark. — başlamak için en iyi iki kaynak) |
| **SCP algoritmaları** | [29] SCvx, [31] GuSTO, [32] adaptif ağırlık, **[33] prox-linear** |
| **SCP uygulamaları** | [34] aerodinamik, **[35] free-final-time**, [36] multi-phase, [37–39] STC, [40, 41] integer |
| **Quadrotor / hipersonik** | [42–47] |
| **Dual quaternion / SPLICE** | [39, 49, 50] |
| **PIPG** | [51–54] |
| **CT-SCvx** | **[55]** (ana makale), [60] passively-safe, [61] GPU, **[62] NMPC**, [63] uçak |
| **Sayısal optimizasyon** | [56] Nocedal & Wright, [57, 58] multiple shooting, [59] error bounds |
| **STL / robustness** | [64, 65] STL temeli, [66] formal methods, [67, 68] MIP, [69–73] pürüzsüzleştirme, [74] locality/masking, **[75] D-GMSR** |
| **Çözücüler** | [76] CVXPY, [77] ECOS, [78] MOSEK |

---

## Değişiklik geçmişi

| Tarih | Ne eklendi |
|---|---|
| — | **Bölüm I (Introduction) — Adım 1 tamamlandı.** Ek olarak: kod deposu analizi (§6), errata (§7), tez entegrasyon planı (§8), terimler sözlüğü (§10), referans haritası (§11). 3 inline SVG. |

> **Sonraki:** Bölüm II.A — Rocket Dynamics. Bu bölüm kodun `rl_dynamics` fonksiyonuyla birebir eşleşir; denklem parçalama (Adım 2) orada tam işleyecektir.
