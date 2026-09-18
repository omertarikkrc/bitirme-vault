---
tags:
  - makale-notu
  - powered-descent-guidance
  - sequential-convex-programming
  - state-triggered-constraints
  - continuous-time-constraint-satisfaction
  - 6-DOF
  - bitirme-tezi
kaynak: "Uzun, S., Açıkmeşe, B., and Carson III, J. M., “Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-Time Compound State-Triggered Constraints,” AIAA SCITECH 2025 Forum, 2025, p. 1895. https://doi.org/10.2514/6.2025-1895"
bolum: "I. Introduction (tam); II.A (tam); II.B (tam); II.C (tam)"
durum: Bolum-II.C-tamamlandi
ilgili:
  - "[[Wang_Song_2018_KAPSAMLI_NOT]]"
  - "[[Angara_1.2_6DOF_Simulink_Modeli]]"
kod-deposu: "https://github.com/sametuzun781/CT-cSTC"
---

# Uzun, Açıkmeşe & Carson (2025) — Kapsamlı Not

> **Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-Time Compound State-Triggered Constraints**

---

## 0. Bu notun kapsamı ve nasıl okunmalı

Bu not, makalenin **Introduction bölümünün tam analizini** ve buna ek olarak **kod deposunun incelenmesini** içerir. Amaç, notu okuyan birinin makaleyi açmadan Introduction'da anlatılan literatür zincirini, kavramları ve bu kavramların tez mimarisine nasıl bağlandığını eksiksiz anlayabilmesidir.

Not **kümülatiftir** — sonraki bölümler (II. PDG Problemi, III. SCP Çözüm Metodu, IV. Sayısal Sonuçlar, V. Sonuç) analiz edildikçe bu dosyanın üzerine eklenecektir.

Yapı:

| Bölüm | İçerik |
|---|---|
| §1 | Künye ve kaynak/kod deposu durumu |
| §2 | Makalenin bir paragraflık özü |
| §3 | Introduction 1/2 — polinom yöntemlerden LCvx'e |
| §4 | Introduction 2/2 — SCP'den D-GMSR'a |
| §5 | Derinleştirilmiş konular (kavramsal netleştirmeler) |
| §6 | Kod deposu analizi |
| §7 | Tespitler, errata, makale–kod tutarsızlıkları |
| §8 | Tez entegrasyonu ve uygulama yol haritası |
| §9 | Açık sorular |
| §10 | Terimler sözlüğü (tüm kısaltmaların açılımı) |
| §11 | Referans haritası — hangi kaynak ne için okunmalı |

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

| Yazar | Kurum | Unvan |
|---|---|---|
| Samet Uzun | University of Washington, Seattle | Doktora öğrencisi, William E. Boeing Dept. of Aeronautics & Astronautics |
| Behçet Açıkmeşe | University of Washington, Seattle | Profesör, AIAA Fellow |
| John M. Carson III | NASA Johnson Space Center | Technical Integration Manager – Precision Landing, NASA STMD, AIAA Fellow |

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

> **Okuma sırası:** Bu bölüm iki kez anlatılıyor. Önce **§3.4.2.0** — tek sayı, somut rakamlar, vektör yok. Kavram oturduktan sonra **Adım 1–6** aynı şeyi genel formda tekrarlıyor. İlk okumada §3.4.2.0 yeterlidir.

##### 3.4.2.0 En basit hali: tek sayı, somut rakamlar

**Kurgu.** Motor en az **4**, en fazla **10** birim itki üretebilir. (Gerçekte 880 kN ve 2200 kN; rakamlar önemli değil.) İtki bir yön taşıdığı için negatif taraf da var. İzin verilen değerler:

```
  ←─────●━━━━━━●░░░░░░░░░░░░●━━━━━━●─────→
      -10     -4     0      4     10
        izin var   YASAK    izin var
```

**Ortada bir DELİK var.** Motor "biraz çalışmaz": ya en az 4 birim verir, ya hiç çalışmaz.

**Delik neden sorun.** Konveks çözücüler bir noktada durup "bir milim şu yöne kaysam daha iyi olur mu?" diye sorar, kayar, tekrar sorar. **Sürekli kayarak** ilerler; zıplayamaz.

Çözücü $T=+4$'te durup asıl iyi çözümün $T=-4$ tarafında olduğunu bilmiyorsa, oraya gitmek için 0'dan geçmesi gerekir — ama 0 yasak. **İki ayrı ada, arada köprü yok.** Tek çare: "önce sağ adayı dene, sonra sol adayı dene."

Ve patlama burada: bu seçim **her zaman düğümünde ayrı ayrı** yapılmalı. $K=15$ düğüm için

$$2^{15} = 32\,768 \text{ ayrı kombinasyon}$$

Düğüm sayısı 30 olursa bir milyarı geçer. Bu, MIP'in kombinatoryal patlamasıdır.

> **Sorun "matematik çirkin" değil. Sorun, deliğin çözücüyü sayısız ayrı seçenek denemeye zorlaması.**

**Numaranın tek cümlelik özü:**

> Yasak olanı, "yasak" olmaktan çıkarıp **"serbest ama pahalı"** yapıyoruz.

**Çim analojisi:**

| Kural | Harita nasıl görünür | Sonuç |
|---|---|---|
| **Eski:** "Çimde yürümek YASAKTIR" | Çim alanı **kesilmiş**, geçilmez boşluk | Etrafından dolaşmak zorunlu — harita parçalı |
| **Yeni:** "Çimde yürüyebilirsin, 100 TL ceza" | Harita **kesintisiz** | Kimse zaten çimden geçmez, ama harita bütün |

Çim hâlâ fiilen kullanılmıyor. Ama harita artık delik içermiyor. **Delik, yasağın kendisinden değil, yasağın haritada boşluk açmasından kaynaklanıyordu.**

**Cezayı nasıl kuruyoruz.** İkinci bir sayı: $\Gamma$. Artık iki kadran var:

| Kadran | Anlamı |
|---|---|
| $T$ | Motorun gerçekten ürettiği itki |
| $\Gamma$ | **Yakıt faturası** — yaktığınız yakıtın karşılığı |

Üç kural:

$$\text{(1)}\quad |T| \le \Gamma \qquad \text{ödediğinizden fazlasını alamazsınız (fazla ödeyebilirsiniz)}$$
$$\text{(2)}\quad 4 \le \Gamma \le 10 \qquad \text{fatura bu bantta olmalı}$$
$$\text{(3)}\quad \dot m = -\alpha\,\Gamma \qquad \text{yakıt } \Gamma \text{ kadar tükenir — } |T| \text{ kadar değil}$$

(3) cezanın ta kendisidir; o olmazsa $\Gamma$'yı şişirmek bedava olur ve numara çöker.

**Somut noktalar.** Her "nokta" artık iki sayıdan oluşuyor: $(T, \Gamma)$.

| Nokta | $T$ | $\Gamma$ | Yakıt yakılan | İtki alınan | Yasal mı? | Optimizasyon seçer mi? |
|---|---|---|---|---|---|---|
| **A** | 8 | 8 | 8 | 8 | ✓ | **Evet** — dürüst, israf yok |
| **B** | 8 | 9 | 9 | 8 | ✓ | Hayır — 1 birim yakıt boşa |
| **C** | 0 | 5 | 5 | **0** | ✓ | Hayır — 5 birim yaktı, itki yok |
| **D** | 8 | 7 | 7 | 8 | ✗ | Yasal değil — (1) ihlal |

**C satırı kilit.** Eski problemde $T=0$ **yasaktı**. Yeni problemde **yasal** ama 5 birim yakıtı çöpe atıyor. Delik böyle kapandı: sıfır noktası artık haritada var, sadece kimse oraya gitmek istemiyor.

**Punchline — orijinal kısıt kendiliğinden geri geliyor.** B noktasına bakın: $(T=8,\Gamma=9)$. Optimizasyon der ki: *"$\Gamma$'yı 8'e düşürebilirim; itki değişmez, 1 birim yakıt kazanırım."* Bu mantık her yerde geçerli — $\Gamma > |T|$ olan hiçbir nokta optimal olamaz.

$$\Gamma^\star = |T^\star| \quad\text{ve}\quad 4 \le \Gamma^\star \le 10 \qquad\Longrightarrow\qquad \boxed{4 \le |T^\star| \le 10}$$

Başladığımız kısıt geri geldi. Hiçbir yerde yazmadık, hiçbir kural zorlamadı — optimizasyon israftan kaçtığı için kendiliğinden oraya oturdu.

> **"Lossless" (kayıpsız) kelimesinin anlamı:** kısıtı çözücüden gizledik, ama çözüm yine de onu sağlıyor. Hiçbir şey kaybetmedik.

<svg viewBox="0 0 680 348" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">
  <text x="60" y="40" font-family="sans-serif" font-size="14" font-weight="bold" fill="currentColor">Tek degisken: T</text>

  <text x="350" y="58" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#A32D2D">orta nokta yasak bolgeye dusuyor</text>
  <path d="M263 100 Q350 68 437 100" fill="none" stroke="#BA7517" stroke-width="1.5"/>

  <rect x="263" y="94" width="174" height="12" rx="2" fill="#E24B4A" fill-opacity="0.18"/>
  <line x1="133" y1="100" x2="263" y2="100" stroke="#639922" stroke-width="4" stroke-linecap="round"/>
  <line x1="437" y1="100" x2="567" y2="100" stroke="#639922" stroke-width="4" stroke-linecap="round"/>

  <text x="198" y="88" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#3B6D11">izin var</text>
  <text x="502" y="88" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#3B6D11">izin var</text>

  <circle cx="263" cy="100" r="4.5" fill="#BA7517"/>
  <circle cx="437" cy="100" r="4.5" fill="#BA7517"/>
  <line x1="344" y1="94" x2="356" y2="106" stroke="#A32D2D" stroke-width="2.5"/>
  <line x1="356" y1="94" x2="344" y2="106" stroke="#A32D2D" stroke-width="2.5"/>

  <text x="133" y="122" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">-10</text>
  <text x="263" y="122" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">-4</text>
  <text x="350" y="122" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">0</text>
  <text x="437" y="122" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">4</text>
  <text x="567" y="122" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">10</text>

  <text x="350" y="144" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#A32D2D">yasak bosluk - cozucu buradan gecemez</text>

  <text x="60" y="178" font-family="sans-serif" font-size="14" font-weight="bold" fill="currentColor">Iki degisken: T ve G</text>

  <line x1="263" y1="196" x2="263" y2="254" stroke="#888780" stroke-width="0.5" stroke-dasharray="3,4"/>
  <line x1="437" y1="196" x2="437" y2="254" stroke="#888780" stroke-width="0.5" stroke-dasharray="3,4"/>

  <line x1="90" y1="320" x2="614" y2="320" stroke="#888780" stroke-width="1"/>
  <line x1="350" y1="330" x2="350" y2="196" stroke="#888780" stroke-width="1"/>
  <text x="620" y="324" font-family="sans-serif" font-size="12" fill="currentColor">T</text>
  <text x="342" y="200" text-anchor="end" font-family="sans-serif" font-size="12" fill="currentColor">G</text>

  <line x1="140" y1="220" x2="572" y2="220" stroke="#378ADD" stroke-width="0.5" stroke-dasharray="5,4"/>
  <line x1="140" y1="280" x2="572" y2="280" stroke="#378ADD" stroke-width="0.5" stroke-dasharray="5,4"/>
  <text x="578" y="224" font-family="sans-serif" font-size="12" fill="currentColor">G=10</text>
  <text x="578" y="284" font-family="sans-serif" font-size="12" fill="currentColor">G=4</text>

  <polygon points="263,280 133,220 567,220 437,280" fill="#639922" fill-opacity="0.16" stroke="#639922" stroke-width="0.5"/>
  <line x1="263" y1="280" x2="133" y2="220" stroke="#639922" stroke-width="3.5"/>
  <line x1="437" y1="280" x2="567" y2="220" stroke="#639922" stroke-width="3.5"/>

  <line x1="263" y1="260" x2="437" y2="260" stroke="#BA7517" stroke-width="1.5"/>
  <circle cx="263" cy="260" r="4.5" fill="#BA7517"/>
  <circle cx="437" cy="260" r="4.5" fill="#BA7517"/>
  <circle cx="350" cy="260" r="5" fill="#639922"/>

  <line x1="356" y1="262" x2="452" y2="300" stroke="#888780" stroke-width="0.5" stroke-dasharray="3,3"/>
  <text x="458" y="304" font-family="sans-serif" font-size="12" fill="#3B6D11">T = 0 artik yasal, sadece israfli</text>
</svg>

**Şekil okuması:** Üstte iki turuncu nokta kümenin içinde, ama aralarındaki orta nokta yasak bölgeye düşüyor — konvekslik testi başarısız. Altta aynı iki nokta, ikinci kadran ($\Gamma$) sayesinde bir yüzey üzerinde duruyor ve aralarındaki her nokta bölgenin içinde. Kalın yeşil kenarlar $\Gamma = |T|$ çizgileri — optimizasyonun oturduğu yer.

##### Genel form (aynı şey, vektörlerle)


##### Adım 1 — Yeni bir karar değişkeni icat et

Problemde şu ana kadar tek bir bilinmeyen vardı: itki vektörü $T \in \mathbb{R}^3$ (her zaman düğümünde 3 bileşen).

Şimdi **her zaman düğümüne bir skaler daha** ekliyoruz: $\Gamma(t) \in \mathbb{R}$. Bu, fiziksel bir büyüklük **değildir** — tıpkı §5.4.9'daki $y$ gibi, yalnızca matematiksel amaçla eklenmiş yapay bir değişkendir.

Maliyet: düğüm başına 3 yerine 4 bilinmeyen. Çok ucuz.

##### Adım 2 — Kısıtı ikiye böl

Nonconvex kısıt:

$$T_{min} \le \|T\|_2 \le T_{max} \qquad \text{(halka — KONVEKS DEĞİL)}$$

yerine iki ayrı kısıt yazılır:

$$\|T\|_2 \le \Gamma \qquad \text{(ikinci-derece koni — KONVEKS ✓)}$$
$$T_{min} \le \Gamma \le T_{max} \qquad \text{(skaler üzerinde kutu — KONVEKS ✓)}$$

**Neden birincisi konveks:** $\|T\|_2 \le \Gamma$ kümesi, $(T,\Gamma)$ uzayında gerçekten bir **konidir** — ucu orijinde olan, yukarı açılan bir dondurma külahı. Koninin **içi** dolu bir kümedir ve konvekstir. "Second-order cone" (ikinci-derece koni) teriminin kaynağı tam olarak budur; SOCP çözücülerinin adı buradan gelir.

**Neden ikincisi konveks:** Tek bir skalerin iki sayı arasında kalması — bir aralık. Aralıklar her zaman konvekstir.

##### Adım 3 — KRİTİK ADIM: $\Gamma$'yı yakıt faturasına bağla

Bu adım atlanırsa numara **çalışmaz.** Kütle tükenme dinamiğinde $\|T\|$ yerine $\Gamma$ yazılır:

$$\dot m = -\alpha_{\dot m}\, \Gamma \qquad\text{(eskiden } \dot m = -\alpha_{\dot m}\|T\| \text{ idi)}$$

> ⚠️ **Neden bu adım şart:** Eğer kütle denklemi $\|T\|$ ile kalsaydı, $\Gamma$'yı şişirmek **bedava** olurdu — optimizasyon $\Gamma = T_{max}$ deyip $T$'yi istediği gibi seçerdi, gevşetme gevşek kalırdı ve losslessness **çökerdi**. Numaranın tüm gücü, "$\Gamma$'nın parasını ödemek zorunda olmaktan" gelir.

##### Adım 4 — Ne oldu geometrik olarak: boyut yükseltme (lifting)

İşin özü budur.

| | Uzay | Küme | Konveks mi |
|---|---|---|---|
| **Önce** | $T \in \mathbb{R}^3$ | Halka (ortası oyuk) | ✗ |
| **Sonra** | $(T,\Gamma) \in \mathbb{R}^4$ | Koni dilimi (dolu) | ✓ |

Yani problemi **bir boyut yukarı taşıdık** ve orada küme konveks hale geldi. Bu, optimizasyonda çok genel bir tekniktir: **lifting** (yükseltme).

**Fatura analojisi:** $\Gamma$ = size **kesilen fatura**, $\|T\|$ = gerçekten **aldığınız mal**.

- Orijinal problem: "faturayla malın birebir eşit olması zorunlu" — bu bir **eşitlik** kısıtıdır, ve eşitlik kısıtları konveks kümeler tanımlamaz.
- Gevşetilmiş problem: "fatura maldan büyük olabilir" — bir **eşitsizlik**, konveks.

Kimse aldığından fazlasına para ödemek istemez. Optimizasyon da istemez. Bu yüzden optimal çözümde fatura = mal, yani $\Gamma^\star = \|T^\star\|$ çıkar — **koninin yüzeyinde.**

##### Adım 5 — Asıl kazanç: eşitliği istemeden eşitliği elde etmek

İstediğimiz küme, koninin **yüzeyinin** $T_{min} \le \Gamma \le T_{max}$ dilimidir:

$$\Gamma = \|T\| \quad\text{(YÜZEY — bir eşitlik kısıtı, konveks değil)}$$

Ama istediğimiz:

$$\Gamma \ge \|T\| \quad\text{(İÇ — bir eşitsizlik kısıtı, konveks)}$$

> **LCvx teoreminin söylediği tek cümle:** *"İç"i iste, "yüzey"i bedava al.*

##### Adım 6 — Projeksiyon kontrolü

Gevşetilmiş kümeyi $T$ uzayına geri izdüşürürsek ne görürüz? **Dolu bir top** ($\|T\| \le T_{max}$) — delik kapanmıştır, yani küme gerçekten büyümüştür. Gevşetme gerçektir.

Ama optimal çözümler koninin yüzeyinde oturduğu için, **onların** izdüşümü tam olarak orijinal halkadır. Yani:

- Fizibil küme: büyüdü (delik doldu)
- Optimal çözüm kümesi: değişmedi

Kayıpsızlığın anlamı budur.

#### 3.4.3 Kayıpsızlık kanıtının sezgisi

Teorem der ki: **optimal çözümde $\|T^\star\| = \Gamma^\star$ zaten kendiliğinden sağlanır.**

Sezgisel gerekçe: Diyelim optimal çözümde bir an için $\|T\| < \Gamma$ olsun. O zaman $\Gamma$'yı biraz düşürebilirdiniz — itki aynı kalır, ama **daha az yakıt yakarsınız**. Bu, çözümün optimal olduğu varsayımıyla çelişir. Demek ki optimalde eşitlik geçerlidir.

Titiz kanıt bunu **PMP (Pontryagin's Maximum Principle — Pontryagin Maksimum Prensibi)** ile yapar; costate'lerden türeyen **primer vector**'ün pozitif ölçülü bir zaman kümesinde sıfırlanamayacağını gösterir. PMP'nin rolü §5.1'de ayrıntılı işlenmiştir.

#### 3.4.4 Mağaza analojisi

Bir mağazada ürünler yalnızca **tam sayı** adet satılıyor: 1, 2 veya 3 alabilirsiniz, 1.5 alamazsınız. Bu nonconvex bir kısıttır.

Numara: "tamam, kesirli de alabilirsin" deyip kısıtı gevşetiyorsunuz. Ama fiyatlandırma öyle kurulmuş ki, **en ucuz çözüm zaten hep tam sayı çıkıyor.** Gevşetme size hiçbir şeye mal olmadı — "kayıpsız" (lossless) tam olarak bu demektir.

#### 3.4.5 Görsel: nonconvex halka → konveks koni

<svg viewBox="0 0 780 395" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">

  <!-- ============ PANEL (a): nonconvex halka ============ -->
  <text x="20" y="24" font-family="sans-serif" font-size="13" font-weight="bold" fill="currentColor">(a) Orijinal kume</text>
  <text x="20" y="43" font-family="sans-serif" font-size="12" fill="currentColor">T_min &lt;= ||T|| &lt;= T_max</text>
  <text x="20" y="62" font-family="sans-serif" font-size="12" font-weight="bold" fill="#f87171">KONVEKS DEGIL</text>

  <!-- halka: fill-rule evenodd ile GERCEK delik -->
  <path d="M 75,200 A 95,95 0 1,0 265,200 A 95,95 0 1,0 75,200 Z
           M 130,200 A 40,40 0 1,0 210,200 A 40,40 0 1,0 130,200 Z"
        fill="#f87171" fill-opacity="0.16" fill-rule="evenodd"/>
  <circle cx="170" cy="200" r="95" fill="none" stroke="#f87171" stroke-width="2"/>
  <circle cx="170" cy="200" r="40" fill="none" stroke="#f87171" stroke-width="2" stroke-dasharray="5,4"/>

  <!-- T_max etiketi -->
  <line x1="237" y1="133" x2="278" y2="106" stroke="#94a3b8" stroke-width="1"/>
  <text x="282" y="103" font-family="sans-serif" font-size="11" fill="currentColor">T_max</text>

  <!-- T_min etiketi -->
  <line x1="198" y1="228" x2="248" y2="258" stroke="#94a3b8" stroke-width="1"/>
  <text x="252" y="262" font-family="sans-serif" font-size="11" fill="currentColor">T_min</text>

  <!-- konvekslik testi: kiris -->
  <line x1="75" y1="200" x2="265" y2="200" stroke="#f59e0b" stroke-width="2.5"/>
  <circle cx="75" cy="200" r="5" fill="#f59e0b"/>
  <circle cx="265" cy="200" r="5" fill="#f59e0b"/>
  <text x="38" y="204" font-family="sans-serif" font-size="11" fill="currentColor">T_a</text>
  <text x="274" y="204" font-family="sans-serif" font-size="11" fill="currentColor">T_b</text>

  <!-- orta nokta: kume disinda -->
  <line x1="162" y1="192" x2="178" y2="208" stroke="#f87171" stroke-width="3"/>
  <line x1="178" y1="192" x2="162" y2="208" stroke="#f87171" stroke-width="3"/>
  <line x1="170" y1="214" x2="170" y2="306" stroke="#94a3b8" stroke-width="1" stroke-dasharray="2,4"/>
  <text x="170" y="326" text-anchor="middle" font-family="sans-serif" font-size="11" font-weight="bold" fill="#f87171">orta nokta kume DISINDA</text>
  <text x="170" y="344" text-anchor="middle" font-family="sans-serif" font-size="10" fill="currentColor">T = 0, yani motor kapali</text>

  <!-- ============ OK ============ -->
  <line x1="345" y1="200" x2="393" y2="200" stroke="#94a3b8" stroke-width="2"/>
  <polygon points="398,200 387,195 387,205" fill="#94a3b8"/>
  <text x="370" y="180" text-anchor="middle" font-family="sans-serif" font-size="11" fill="currentColor">slack degiskeni</text>
  <text x="370" y="224" text-anchor="middle" font-family="sans-serif" font-size="11" fill="currentColor">G ekle</text>

  <!-- ============ PANEL (b): konveks koni dilimi ============ -->
  <text x="430" y="24" font-family="sans-serif" font-size="13" font-weight="bold" fill="currentColor">(b) Gevsetilmis kume</text>
  <text x="430" y="43" font-family="sans-serif" font-size="12" fill="currentColor">||T|| &lt;= G   ve   T_min &lt;= G &lt;= T_max</text>
  <text x="430" y="62" font-family="sans-serif" font-size="12" font-weight="bold" fill="#4ade80">KONVEKS</text>

  <!-- eksenler -->
  <line x1="430" y1="320" x2="756" y2="320" stroke="#94a3b8" stroke-width="1.5"/>
  <line x1="580" y1="330" x2="580" y2="120" stroke="#94a3b8" stroke-width="1.5"/>
  <text x="760" y="324" font-family="sans-serif" font-size="11" fill="currentColor">T</text>
  <text x="570" y="113" font-family="sans-serif" font-size="11" fill="currentColor">G</text>

  <!-- T_min / T_max seviyeleri -->
  <line x1="437" y1="185" x2="714" y2="185" stroke="#60a5fa" stroke-width="1.2" stroke-dasharray="5,4"/>
  <line x1="437" y1="265" x2="714" y2="265" stroke="#60a5fa" stroke-width="1.2" stroke-dasharray="5,4"/>
  <text x="719" y="189" font-family="sans-serif" font-size="11" fill="currentColor">T_max</text>
  <text x="719" y="269" font-family="sans-serif" font-size="11" fill="currentColor">T_min</text>

  <!-- konveks bolge (yamuk) -->
  <polygon points="525,265 445,185 715,185 635,265" fill="#4ade80" fill-opacity="0.18" stroke="#4ade80" stroke-width="1.5"/>
  <!-- G = |T| kenarlari (kalin) -->
  <line x1="525" y1="265" x2="445" y2="185" stroke="#4ade80" stroke-width="4"/>
  <line x1="635" y1="265" x2="715" y2="185" stroke="#4ade80" stroke-width="4"/>

  <!-- konvekslik testi: kiris -->
  <line x1="470" y1="205" x2="660" y2="235" stroke="#f59e0b" stroke-width="2.5"/>
  <circle cx="470" cy="205" r="5" fill="#f59e0b"/>
  <circle cx="660" cy="235" r="5" fill="#f59e0b"/>
  <circle cx="565" cy="220" r="5" fill="#4ade80"/>

  <text x="430" y="352" font-family="sans-serif" font-size="11" font-weight="bold" fill="#4ade80">Kalin kenar: G = |T| - optimal cozum burada oturur</text>
  <text x="430" y="371" font-family="sans-serif" font-size="11" fill="currentColor">Her kiris kume icinde kalir - konvekslik testi gecti</text>
</svg>

**Şekil okuması:** (a) panelinde turuncu doğru, kümenin iki noktasını birleştiren bir kiriştir; orta noktası oyuğun içine, yani kümenin dışına düşer — konvekslik testi başarısız. (b) panelinde ise $\Gamma$ ekseni eklenerek küme $(T,\Gamma)$ uzayında bir "dondurma külahı" kesitine dönüşür; her kiriş küme içinde kalır. LCvx teoremi, optimal çözümün yeşil $\Gamma = |T|$ kenarında oturduğunu söyler — yani gevşetme bedava olmuştur.

### 3.5 LCvx'in genişletilmesi

LCvx daha sonra şu yönlerde genişletilmiştir. Her biri gerçek bir mühendislik ihtiyacına karşılık gelir:

| Genişletme | Kaynak | İhtiyaç |
|---|---|---|
| Genel nonconvex kontrol kümeleri | [13] | Sadece itki büyüklüğü değil, daha genel kontrol kısıtları |
| Minimum-error landing | [14] | Hedefe tam inilemiyorsa en az sapmayla inme |
| Thrust pointing constraints | [15] | İtki vektörünün belirli bir koni içinde kalması |
| **Afin durum kısıtları** | [16] | Düz duvar tipi durum kısıtları |
| **Kuadratik durum kısıtları** | [17] | Elipsoit/küre tipi durum kısıtları |
| Nonlineer dinamikler | [18] | Doğrusal olmayan hareket denklemleri |
| Binary (ikili) kısıtlar | [19] | Ayrık/mantıksal seçimler |

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

SCP'nin her iterasyonunda bir **konveks alt-problem** çözülür. Bu alt-problemi kimin çözdüğü, toplam hızı doğrudan belirler — kodda ölçülen zamanın ~%90'ı buradadır (§6.10).

#### 4.4.1 IPM — ikinci-derece yöntem

**IPM (Interior Point Method — İç Nokta Yöntemi)** [9, 10].

**Çekirdek fikir — bariyer (engel) fonksiyonu:** Kısıtları "sert duvar" olarak uygulamak yerine, sınıra yaklaştıkça sonsuza giden bir ceza terimi eklenir. Örneğin $g(x) \le 0$ kısıtı için maliyete $-\mu \ln(-g(x))$ eklenir. Sonra $\mu$ kademeli olarak küçültülür — yani duvar giderek keskinleşir. Çözüm hep kümenin **iç**inden yaklaşır; adı buradan gelir.

**Her iterasyonda ne yapılır:** Gradyan **ve eğrilik** (Hessian) kullanılarak bir KKT lineer sistemi kurulur ve **matris çarpanlamasıyla** çözülür. Bu, pahalı olan adımdır.

**Analoji:** Çok dikkatli bir dağcı. Her adımdan önce teodolit kurup çevreyi ölçüyor, eğimi ve eğriliği hesaplıyor, sonra tam doğru yere büyük bir adım atıyor. **Az adım, her biri pahalı.**

| | |
|---|---|
| İterasyon sayısı | Tipik olarak 10–50 (çok az) |
| İterasyon maliyeti | Yüksek — matris çarpanlaması |
| Hassasiyet | Çok yüksek |
| Bellek | Dinamik ayırma |
| Çalışma süresi | **Değişken** |
| Uçuş bilgisayarına uygunluk | Zayıf — sertifikasyon zor |

#### 4.4.2 PIPG — birinci-derece yöntem

**PIPG (Proportional-Integral Projected Gradient — Oransal-İntegral İzdüşümlü Gradyan)** [51, 52].

**Çekirdek fikir üç parçadan oluşur:**

1. **Projected Gradient (izdüşümlü gradyan):** Maliyetin gradyanı yönünde küçük bir adım at. Kümenin dışına çıktıysan, en yakın noktaya **geri izdüşür** (projection). Norm topu veya kutu gibi basit kümelere izdüşüm kapalı formülle, anında hesaplanır.
2. **Proportional-Integral:** Kısıt ihlalini bir **hata sinyali** gibi ele alıp, dual (eş) değişkenleri klasik bir **PI kontrolcü** yapısıyla günceller. Oransal terim anlık ihlale, integral terimi birikmiş ihlale tepki verir.
3. **Matris çarpanlaması yok** — yalnızca matris-**vektör** çarpımları.

> Bu, kontrol mühendisliği açısından şık bir detaydır: algoritma, optimizasyon kısıtlarını "regüle edilecek bir hata" gibi görüp literatürün en tanıdık kontrolcüsünü (PI) bu işe koşuyor.

**Analoji:** Ayağının altındaki eğimi hissederek yürüyen bir yürüyüşçü. Ölçüm aleti yok, hesap yok. Duvara çarpınca duvar boyunca kayıyor. **Çok adım, her biri çok ucuz.**

| | |
|---|---|
| İterasyon sayısı | Yüzler–binler (çok fazla) |
| İterasyon maliyeti | Çok düşük — sadece matris-vektör |
| Hassasiyet | Orta |
| Bellek | Statik, küçük |
| Çalışma süresi | **Sabit** (iterasyon sayısı sabitlenebilir) |
| Uçuş bilgisayarına uygunluk | **Güçlü** — basit kod, GPU'ya paralelleşir |

#### 4.4.3 Karşılaştırma

| | IPM | PIPG |
|---|---|---|
| Mertebe | İkinci (gradyan + Hessian) | Birinci (sadece gradyan) |
| Pahalı adım | Matris çarpanlaması | Yok |
| Az mı çok mu iterasyon | Az, pahalı | Çok, ucuz |
| Doğruluk | Yüksek | Orta |
| Öngörülebilir süre | Hayır | **Evet** |
| Tipik kullanım | Çevrimdışı, yüksek hassasiyet | **Gerçek-zamanlı, gemi üstü** |

#### 4.4.4 Neden PIPG özellikle SCP'ye yakışıyor

Kritik gözlem: SCP zaten alt-problemi **defalarca** çözer (kodda `ite = 85`). Her alt-problem, bir sonraki iterasyonda **yeniden lineerleştirilecek** bir yaklaşımdır.

> Dolayısıyla alt-problemi 12 haneli hassasiyetle çözmenin anlamı yoktur — zaten yaklaşık bir modeli çözüyorsunuz. Orta hassasiyet yeterlidir.
>
> **Birinci-derece yöntemlerin SCP ile bu kadar iyi eşleşmesinin sebebi budur:** IPM'in sunduğu ekstra hassasiyet, bu bağlamda **israftır**.

#### 4.4.5 Literatürdeki uygulama sonuçları

| Uygulama | Kaynak | Sonuç |
|---|---|---|
| 3-DoF LCvx problemine | [53] | Çözüm hızında ciddi iyileşme |
| Dual quaternion 6-DoF PDG'ye | [54] | Gerçek-zamanlı performansta belirgin artış |

> ⚠️ **Bu makale PIPG kullanmaz.** CVXPY + MOSEK/CLARABEL (yani IPM) kullanır; PIPG yalnızca "verimli implementasyonlar için kullanılabilir" diye anılır. Kodda `PIPG` kelimesi **hiç geçmez** (§5.3).

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

> **Ek not — neden bu iki adımda kaçınılmaz çıkıyor:** Denklemi $\gamma$ **düşeyden** ölçülüyor varsayarak tek adımda türetmeye çalışırsanız $d\le\tan(\gamma_{max})h$ çıkar — makalenin yazdığı $\tan(\gamma_{max})d\le h$'nin **tersi.** Ama $\gamma$'yı **yataydan** (klasik LCvx literatüründeki "glideslope angle" tanımı, örn. Açıkmeşe & Blackmore 2010) ölçülüyor kabul edince denklem **tek adımda, hiç ters çevirmeden** çıkıyor: $\tan\gamma=h/d \Rightarrow \tan(\gamma_{max})d\le h$. Yani makalenin **denklemi** yatay-konvansiyonu varsayıyor, ama **Tablo 3 + Fig.2 + kod** düşey-konvansiyonla tutarlı — makalenin kendi içinde iki farklı yerin birbiriyle örtüşmediği, tek satırlık bir yazım hatasından daha ince bir iç tutarsızlık.

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

### 7.9 ⚠️ Model yapısal olarak roll momenti üretemiyor (makalede tartışılmıyor)

Tek gimballi motor ve eksenel simetrik atalet varsayımının, makalenin hiç değinmediği bir sonucu var.

**İtki momenti.** $r_{cm,B}=(0,0,-14)$ ve $T_B=(T_x,T_y,T_z)$ için:

$$r_{cm,B} \times T_B = \begin{pmatrix} 0 \\ 0 \\ -14 \end{pmatrix} \times \begin{pmatrix} T_x \\ T_y \\ T_z \end{pmatrix} = \begin{pmatrix} 14\,T_y \\ -14\,T_x \\ \mathbf{0} \end{pmatrix}$$

Üçüncü bileşen **sıfır** — gimballi tek motor, gövde ekseni etrafında (roll) moment üretemez.

**Aerodinamik momenti.** $r_{cp,B}=(0,0,3)$ — aynı yapı, yine $z$ bileşeni sıfır.

**Jiroskopik terim.** $J_B = m\cdot\mathrm{diag}(60,60,1.5)$ eksenel simetrik olduğu için:

$$\big[\omega_B \times (J_B\omega_B)\big]_z = p\,(60m)\,q - q\,(60m)\,p = 0$$

**Sonuç:**

$$\dot\omega_z \equiv 0 \quad\text{her zaman}$$

Roll hızı başlangıçta sıfırsa ($\omega_{Bi}=(0,0,0)$, Tablo 3) sonsuza kadar sıfır kalır. Bu modelde roll **hiç kontrol edilmiyor ve hiç değişmiyor** — çünkü onu değiştirecek hiçbir mekanizma yok.

> ⚠️ Gerçek bir roket böyle davranmaz: rüzgâr, imalat asimetrisi veya kanatçık etkisi roll üretir. Roll kontrolü için ayrı aktüatör gerekir — **RCS itici, vernier motor, veya grid fin.**
>
> **Tez kararı (alındı):** İlk aşamada roll ihmal edilecek ve bu **açık bir varsayım olarak** tez metnine yazılacak. Modele roll aktüatörü eklemek ileri aşama işi.

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
- [ ] $v_f$ değeri (küçük kalıntı hız) Angara 1.2 iniş takımı toleransına göre yeniden seçilmeli mi? (§14.4.1)
- [ ] Çoklu-site mimarisi için $X$ kümesinin $r_f$ parametrik hale getirilmesi ne zaman ele alınacak? (§14.5, §12.12.5)

### 9.2 Analiz sırası — sonraki adımlar

- [x] Bölüm II.A — Rocket Dynamics
- [x] Bölüm II.B — kısıtlar
- [x] Bölüm II.C — sınır koşulları
- [ ] **Bölüm II.D — compound STC'ler** (sıradaki — makalenin asıl özgün katkısı, boresight line-of-sight kısıtının tam detayı)
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

---

## 12. Bölüm II.A — Rocket Dynamics

> **Durum:** Adım 1, Bölüm 1/3 tamamlandı. Bu bölüm kodun `rl_dynamics` fonksiyonuyla birebir örtüşür.

### 12.1 İki referans çerçevesi

| Çerçeve | Nereye bağlı | Ne kolay olur |
|---|---|---|
| **I** (inertial) | Yere / iniş noktasına sabit, $z$ yukarı | Yerçekimi sabit: $g_I=(0,0,-g_0)$. Konum ve hız burada anlamlı |
| **B** (body) | Rokete bağlı, **onunla döner** | Motor hep aynı yerde. Atalet momenti sabit. Aerodinamik gövdeye göre tanımlı |

**Gemi analojisi:** Kaptan "kuzeye git" der (sabit çerçeve) ama "iskele tarafındaki halatı çek" der (gemiye bağlı çerçeve). Gemi döndüğünde "iskele" dünyaya göre başka yönü gösterir, tayfa için hep aynı taraftır.

Çevirmen: $C_{I\leftarrow B}$, quaternion'dan hesaplanır. Okunuşu sağdan sola: "B'den I'ya". Bu yüzden itki hem $T_B$ (gövdede tanımlı, motor gövdeye vidalı) hem $C_{I\leftarrow B}T_B$ (Newton I'da geçerli) olarak görünür.

### 12.2 Durum vektörü — 14 fiziksel boyut

$$x := (m,\; r_I,\; v_I,\; q_{B\leftarrow I},\; \omega_B)$$

| Bileşen | Boyut | Çerçeve | Ne | Birim |
|---|---|---|---|---|
| $m$ | 1 | — | Kütle | kg |
| $r_I$ | 3 | I | Konum | m |
| $v_I$ | 3 | I | Hız | m/s |
| $q_{B\leftarrow I}$ | 4 | — | Tutum (quaternion) | boyutsuz |
| $\omega_B$ | 3 | B | Açısal hız | rad/s |
| | **14** | | | (+1 CTCS sayacı → $n_x=15$, bkz. §5.4) |

**Kütle neden durum, parametre değil:** Uçuş boyunca 100 t → 90 t değişiyor ve $F=ma$'da paydada. Aynı itki, hafifleyen rokete daha büyük ivme verir; sonda roket çok daha atik olur.

Kodda atalet momenti de kütleyle ölçekleniyor:

```python
J_B_inv_m = J_B_inv_pre / x[0]
J_B_m     = J_B_pre * x[0]
```

> ⚠️ Kod `r_cm` sabit alıyor (CG kayması yok). Sizin §4.2 kararınız daha ayrıntılı — bkz. §7.8.

### 12.3 Euler açıları, tekillik ve quaternion tercihi

#### 12.3.1 Euler açıları — 3-2-1 sırası

| Sıra | Açı | Eksen | Türkçe |
|---|---|---|---|
| 1 | $\psi$ | $z$ | Sapma (yaw) |
| 2 | $\theta$ | $y$ | Yunuslama (pitch) |
| 3 | $\phi$ | $x$ | Yuvarlanma (roll) |

$$C_{B\leftarrow I} = R_x(\phi)\,R_y(\theta)\,R_z(\psi)$$

Açık hali ($c=\cos$, $s=\sin$):

$$C_{B\leftarrow I} = \begin{bmatrix}
c\theta\, c\psi & c\theta\, s\psi & -s\theta \\
s\phi\, s\theta\, c\psi - c\phi\, s\psi & s\phi\, s\theta\, s\psi + c\phi\, c\psi & s\phi\, c\theta \\
c\phi\, s\theta\, c\psi + s\phi\, s\psi & c\phi\, s\theta\, s\psi - s\phi\, c\psi & c\phi\, c\theta
\end{bmatrix}$$

#### 12.3.2 Tekillik nerede: kinematik denklemler

Asıl sorun matriste değil, gövde açısal hızı $\omega_B=(p,q,r)$ ile Euler açı hızları arasındaki ilişkidedir:

$$\begin{bmatrix}\dot\phi \\ \dot\theta \\ \dot\psi\end{bmatrix} =
\begin{bmatrix}
1 & \sin\phi\,\tan\theta & \cos\phi\,\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \dfrac{\sin\phi}{\cos\theta} & \dfrac{\cos\phi}{\cos\theta}
\end{bmatrix}
\begin{bmatrix}p \\ q \\ r\end{bmatrix}$$

Birinci ve üçüncü satırda $\tan\theta$ ve $1/\cos\theta$ var. $\theta \to \pm 90°$ olduğunda:

$$\cos(90°)=0 \;\Longrightarrow\; \tan(90°)=\infty,\quad \frac{1}{\cos(90°)}=\infty$$

$\dot\phi$ ve $\dot\psi$ **tanımsız** olur. Jiroskop çalışıyor, $\omega_B$ sonlu — patlayan şey koordinat dönüşümü.

> Sayısal olarak $\theta=89.9°$ bile kötüdür: $1/\cos(89.9°) \approx 573$. Küçük ölçüm gürültüsü 573 katına çıkar.

#### 12.3.3 Ne kaybolduğunu görmek: matrisin çöküşü

$\theta=90°$ koyalım ($c\theta=0$, $s\theta=1$):

$$C_{B\leftarrow I}\Big|_{\theta=90°} = \begin{bmatrix}
0 & 0 & -1 \\
s\phi\, c\psi - c\phi\, s\psi & s\phi\, s\psi + c\phi\, c\psi & 0 \\
c\phi\, c\psi + s\phi\, s\psi & c\phi\, s\psi - s\phi\, c\psi & 0
\end{bmatrix}
= \begin{bmatrix}
0 & 0 & -1 \\
\sin(\phi-\psi) & \cos(\phi-\psi) & 0 \\
\cos(\phi-\psi) & -\sin(\phi-\psi) & 0
\end{bmatrix}$$

**Matris yalnızca $(\phi-\psi)$ farkına bağlı.** Somut sonuç: $(\phi,\psi)=(30°,10°)$ ile $(80°,60°)$ **tamamen aynı yönelimi** verir. İki parametre, tek bilgi. **Bir serbestlik derecesi kayboldu.**

<svg viewBox="0 0 680 336" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">
  <defs><marker id="arrEU" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/></marker></defs>

  <text x="190" y="44" text-anchor="middle" font-family="sans-serif" font-size="14" font-weight="bold" fill="currentColor">theta = 0</text>
  <line x1="190" y1="170" x2="190" y2="84" stroke="#378ADD" stroke-width="2.5" marker-end="url(#arrEU)"/>
  <text x="190" y="74" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#185FA5">psi sapma</text>
  <line x1="190" y1="170" x2="282" y2="170" stroke="#639922" stroke-width="2.5" marker-end="url(#arrEU)"/>
  <text x="290" y="174" font-family="sans-serif" font-size="12" fill="#3B6D11">theta</text>
  <line x1="190" y1="170" x2="122" y2="230" stroke="#7F77DD" stroke-width="2.5" marker-end="url(#arrEU)"/>
  <text x="108" y="246" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#534AB7">phi</text>
  <circle cx="190" cy="170" r="4" fill="#5F5E5A"/>
  <text x="190" y="290" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">uc eksen dik - 3 bagimsiz donus</text>

  <text x="500" y="44" text-anchor="middle" font-family="sans-serif" font-size="14" font-weight="bold" fill="currentColor">theta = 90</text>
  <rect x="493" y="80" width="14" height="182" rx="4" fill="#EF9F27" fill-opacity="0.28"/>
  <line x1="500" y1="170" x2="500" y2="84" stroke="#378ADD" stroke-width="2.5" marker-end="url(#arrEU)"/>
  <text x="500" y="74" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#185FA5">psi sapma</text>
  <line x1="500" y1="170" x2="592" y2="170" stroke="#639922" stroke-width="2.5" marker-end="url(#arrEU)"/>
  <text x="600" y="174" font-family="sans-serif" font-size="12" fill="#3B6D11">theta</text>
  <line x1="500" y1="170" x2="500" y2="256" stroke="#7F77DD" stroke-width="2.5" marker-end="url(#arrEU)"/>
  <text x="516" y="250" font-family="sans-serif" font-size="12" fill="#534AB7">phi</text>
  <circle cx="500" cy="170" r="4" fill="#5F5E5A"/>
  <text x="500" y="290" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#854F0B">phi ve psi ayni dogru uzerinde</text>
  <text x="500" y="310" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">1 serbestlik derecesi kayboldu</text>
</svg>

#### 12.3.4 Neden kaçınılmaz: topolojik sebep

**Başka bir Euler sırası seçmek sorunu çözmez.** 3-2-1 yerine 3-1-3 seçerseniz tekillik başka bir yönelime kayar, yok olmaz.

Sebebi: dönmeler uzayı $SO(3)$, üç parametreyle **tekilliksiz kaplanamaz.** Bu bir teoremdir, seçim meselesi değil.

**Harita analojisi:** Küreyi tek bir düz kâğıda, yırtmadan veya sonsuza germeden seremezsiniz. Mercator kutupları sonsuza gerer; başka projeksiyon bozulmayı başka yere taşır ama yok edemez. Küre ile düzlem topolojik olarak farklı nesnelerdir.

Euler açıları $SO(3)$'ün Mercator projeksiyonudur. Quaternion ise küreyi kâğıda sermek yerine **bir boyut yukarı çıkıp küre olarak bırakır**: 4 sayı + 1 kısıt = birim 3-küre $S^3$.

> Bu, §3.4.2'deki slack değişkeni numarasıyla **aynı stratejidir**: tekil/nonkonveks bir yapıyı bir boyut yukarı taşıyarak düzeltmek.

#### 12.3.5 Quaternion kinematiği — karşılaştırma

$$\dot q_{B\leftarrow I} = \tfrac{1}{2}\,\Omega(\omega_B)\, q_{B\leftarrow I}$$

| | Euler | Quaternion |
|---|---|---|
| Parametre sayısı | 3 | 4 (+1 kısıt) |
| Kinematik denklem | Trigonometrik, **bölme içeriyor** | **Bilineer**, bölme yok |
| Tekillik | $\theta=\pm90°$ | **Yok** |
| Hesap maliyeti | sin/cos/tan | Sadece çarpma-toplama |
| Türev alınabilirlik | Tekillikte bozulur | Her yerde pürüzsüz |

Bölme olmaması SCP için ayrıca kritiktir: Jacobian `jacfwd` ile otomatik alınır; $1/\cos\theta$ gibi bir terim tekilliğe yaklaşırken Jacobian'ı da patlatırdı.

**Bonus — norm analitik olarak korunur.** $\Omega$ çarpık simetriktir ($\Omega^\top=-\Omega$), dolayısıyla:

$$\frac{d}{dt}\|q\|^2 = 2\,q^\top \dot q = q^\top \Omega\, q = 0$$

(çarpık simetrik $A$ için $x^\top A x = 0$)

> Yani birim norm **denklemden** kaymaz; kayma tamamen sayısal entegrasyon hatasıdır. §4.2'deki normalizasyon kararı ($q \leftarrow q/\|q\|$) gerçek bir fiziksel sürüklenmeyi değil, yalnızca yuvarlama hatasını temizler — bu yüzden ucuz ve zararsız bir düzeltmedir.

#### 12.3.6 Bu makale için neden kritik

Senaryo: roket **90° yatık** başlıyor ($q_i=(\tfrac{\sqrt2}{2},\tfrac{\sqrt2}{2},0,0)$), dik bitiyor ($q_f=(1,0,0,0)$). Manevra tutum uzayında tam 90°'lik bir süpürmedir — uygun bir Euler sırasında tekillik noktasının tam üstünden geçer. Üstelik optimizasyon bu yörüngeyi *arar*, yani tekillik bölgesine defalarca girip çıkar.

> **Quaternion burada tercih değil, zorunluluktur.** Euler açılarıyla bu problem sayısal olarak çözülemezdi.

### 12.4 Kontrol vektörü — 5 boyut, ikisi dinamiğe girmiyor

$$u := (T,\; \delta^e,\; \phi^e,\; \delta^b,\; \phi^b)$$

| Bileşen | Ne | Dinamiğe girer mi? |
|---|---|---|
| $T$ | İtki büyüklüğü | ✓ |
| $\delta^e$ | Motor gimbal **sapma** açısı | ✓ |
| $\phi^e$ | Motor gimbal **azimut** açısı | ✓ |
| $\delta^b$ | Boresight sapma açısı | ✗ |
| $\phi^b$ | Boresight azimut açısı | ✗ |

Boresight (görüş ekseni), iniş alanını görmesi gereken sensörün nişan yönüdür. **Aktüatör değildir** — kuvvet veya moment üretmez.

Kodda doğrulandı: `T_v` yalnızca `u[0], u[1], u[2]` kullanır; `u[3], u[4]` sadece line-of-sight STC'sinde geçer.

> Boresight açıları **fiziği değil, gözlem kısıtını** kontrol eder. "Her kontrol girdisi bir kuvvet üretir" beklentisini bozan bir tasarım, ama optimizasyon açısından meşru: nişan yönü de bir karar değişkenidir.

#### 12.4.1 İtki vektörü — küresel koordinat parametrizasyonu

$$T_B := T\begin{bmatrix} \sin(\delta^e)\cos(\phi^e) \\ \sin(\delta^e)\sin(\phi^e) \\ \cos(\delta^e)\end{bmatrix}$$

$\delta^e$ = gövde ekseninden sapma miktarı, $\phi^e$ = eksen etrafında hangi yöne sapıldığı.

<svg viewBox="0 0 680 320" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">
  <defs><marker id="arrGB" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/></marker></defs>

  <text x="60" y="40" font-family="sans-serif" font-size="14" font-weight="bold" fill="currentColor">Yandan gorunus</text>
  <line x1="205" y1="215" x2="205" y2="56" stroke="#888780" stroke-width="1" stroke-dasharray="5,4"/>
  <text x="205" y="48" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">z_B</text>
  <polygon points="182,62 205,40 228,62" fill="#B4B2A9" fill-opacity="0.35" stroke="#5F5E5A" stroke-width="0.5"/>
  <rect x="182" y="62" width="46" height="142" rx="6" fill="#B4B2A9" fill-opacity="0.35" stroke="#5F5E5A" stroke-width="0.5"/>
  <circle cx="205" cy="108" r="5" fill="#5F5E5A"/>
  <text x="172" y="112" text-anchor="end" font-family="sans-serif" font-size="12" fill="currentColor">CM</text>
  <line x1="205" y1="116" x2="205" y2="196" stroke="#378ADD" stroke-width="1" stroke-dasharray="3,3"/>
  <text x="172" y="162" text-anchor="end" font-family="sans-serif" font-size="12" fill="currentColor">r_cm</text>
  <line x1="205" y1="204" x2="256" y2="95" stroke="#888780" stroke-width="0.5" stroke-dasharray="4,4"/>
  <line x1="205" y1="204" x2="154" y2="95" stroke="#888780" stroke-width="0.5" stroke-dasharray="4,4"/>
  <text x="262" y="99" font-family="sans-serif" font-size="12" fill="currentColor">delta_max</text>
  <line x1="205" y1="204" x2="250" y2="88" stroke="#E24B4A" stroke-width="2.5" marker-end="url(#arrGB)"/>
  <circle cx="205" cy="204" r="4" fill="#5F5E5A"/>
  <text x="256" y="70" font-family="sans-serif" font-size="12" fill="#A32D2D">T_B</text>
  <path d="M 205 134 A 70 70 0 0 1 228 138" fill="none" stroke="#7F77DD" stroke-width="1.5"/>
  <text x="220" y="126" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#3C3489">delta</text>

  <text x="400" y="40" font-family="sans-serif" font-size="14" font-weight="bold" fill="currentColor">Eksen boyunca gorunus</text>
  <circle cx="520" cy="150" r="72" fill="none" stroke="#888780" stroke-width="0.5" stroke-dasharray="4,4"/>
  <text x="520" y="244" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">delta_max siniri</text>
  <line x1="520" y1="150" x2="602" y2="150" stroke="#888780" stroke-width="1" marker-end="url(#arrGB)"/>
  <line x1="520" y1="150" x2="520" y2="68" stroke="#888780" stroke-width="1" marker-end="url(#arrGB)"/>
  <text x="610" y="154" font-family="sans-serif" font-size="12" fill="currentColor">x_B</text>
  <text x="520" y="60" text-anchor="middle" font-family="sans-serif" font-size="12" fill="currentColor">y_B</text>
  <line x1="520" y1="150" x2="557" y2="106" stroke="#E24B4A" stroke-width="2.5" marker-end="url(#arrGB)"/>
  <circle cx="520" cy="150" r="4" fill="#5F5E5A"/>
  <text x="562" y="100" font-family="sans-serif" font-size="12" fill="#A32D2D">T_B izi</text>
  <path d="M 554 150 A 34 34 0 0 0 542 124" fill="none" stroke="#7F77DD" stroke-width="1.5"/>
  <text x="556" y="138" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#3C3489">phi</text>
</svg>

### 12.5 Kritik gözlem: nonkonvekslik nereye gitti

$$\|T_B\|_2^2 = T^2\big(\underbrace{\sin^2\delta^e\cos^2\phi^e + \sin^2\delta^e\sin^2\phi^e}_{\sin^2\delta^e} + \cos^2\delta^e\big) = T^2
\qquad\Longrightarrow\qquad \boxed{\|T_B\|_2 = T}$$

**Gimbal açıları büyüklüğü değiştirmez, sadece yönü değiştirir.** Dolayısıyla itki kısıtı $T_{min} \le T \le T_{max}$ artık bir **halka değil, tek skaler üzerinde aralık** — konveks. Slack değişkeni ve LCvx gerekmez.

Kodda bu sadeleştirme doğrudan kullanılmış:

```python
f = f.at[0].set(- params['alpha_m'] * u[0])    # ||T_B|| yerine dogrudan T
```

| | Wang & Song / LCvx | Bu makale |
|---|---|---|
| Kontrol girdisi | $T \in \mathbb{R}^3$ | $(T, \delta^e, \phi^e)$ |
| İtki alt sınırı | **Halka — nonkonveks** | **Aralık — konveks** |
| Çözüm | Slack + LCvx teoremi | Gerekmiyor |
| Bedeli | Losslessness kanıtı (kırılgan) | Dinamik **trigonometrik** oldu |

> **Nonkonvekslik yok olmadı — yer değiştirdi.** Kısıtlardan çıkıp dinamiğin içine girdi. Ama SCP zaten dinamiği lineerleştiriyor, yani ek maliyeti yok.
>
> ⭐ **Tez metninde tartışılmaya değer tasarım tercihi: nonkonveksliği, onu zaten ele alan bileşene taşımak.**

### 12.6 Kodla eşleşme (Bölüm 1/3 kapsamı)

```python
CBI = CBI_fcn(x[7:11])        # quaternion -> C_{B<-I}
CIB = CBI.transpose()         # C_{I<-B} = C_{B<-I}^T

T_v = jnp.array([ u[0] * jnp.sin(u[1]) * jnp.cos(u[2]),     # T sin d cos p
                  u[0] * jnp.sin(u[1]) * jnp.sin(u[2]),     # T sin d sin p
                  u[0] * jnp.cos(u[1]) ])                   # T cos d
```

`CIB = CBI.transpose()` — dönme matrisleri **ortogonaldir**, tersi = devriği. Ters matris hesabına gerek yok: 9 sayının yerini değiştirmek yeterli. Sayısal olarak bedava ve hatasız.


### 12.7 Dinamiğin beş satırı

#### 12.7.1 Kütle: $\dot m = -\alpha_{\dot m}\,T$

$$\alpha_{\dot m} := \frac{1}{I_{sp}\, g_0}$$

**Fiziksel köken — roket denklemi.** Motor, yakıtı $v_e$ egzoz hızıyla atarak itki üretir:

$$T = -\dot m\, v_e \quad\Longrightarrow\quad \dot m = -\frac{T}{v_e}$$

$I_{sp}$ (specific impulse — özgül itki) motorun verimlilik ölçüsüdür, $v_e = I_{sp}\,g_0$ ile tanımlıdır. Buradaki $g_0 = 9.806$ m/s² **gerçek yerçekimi değil**, yalnızca birim dönüştürücüdür.

**Sezgi:** $I_{sp}$ büyükse motor az yakıtla çok itki üretir. $I_{sp}=330$ s, tipik sıvı yakıtlı motor için makul.

**Neden $\|T_B\|$ değil $T$:** §12.5'te türetildiği gibi $\|T_B\| = T$ — gimbal açıları büyüklüğü değiştirmiyor.

```python
f = f.at[0].set(- params['alpha_m'] * u[0])
```

#### 12.7.2 Konum: $\dot r_I = v_I$

Tanım gereği doğru — fizik yok, muhasebe var.

**Dikkat edilecek nokta:** Bu satır hem $r_I$ hem $v_I$'yı **aynı I çerçevesinde** tutuyor. Biri I'da diğeri B'de olsaydı önce çerçeve dönüşümü gerekirdi. Denklemi bu kadar sade tutan şey bu tasarım tercihi.

```python
f = f.at[1:4].set(x[4:7])
```

> **JAX sözdizimi notu:** JAX dizileri **değiştirilemez (immutable)** — `f[1:4] = ...` yazılamaz, çünkü otomatik türev (`jacfwd`) ve JIT derleme, fonksiyonların "temiz" (pure) olmasını gerektirir. `f.at[1:4].set(...)` bunun yerine **değiştirilmiş bir kopya** üretir; `f = ...` ile eski değişkenin üzerine yazılır.
>
> İndeksler: $f[i]$ her zaman $\dot x[i]$'yi temsil eder. `f.at[1:4]` = $\dot r_I$ (konumun türevi), `x[4:7]` = $v_I$ (hız). Sağ tarafta hiçbir hesap yok — sadece dilim kopyalanıyor, çünkü bu bir fizik yasası değil bir tanım.

#### 12.7.3 Hız: kuvvetlerin toplamı

$$\dot v_I = \frac{1}{m}\, C_{I\leftarrow B}\big(T_B + A_B\big) + g_I$$

Newton II ($a = F/m$), üç kuvvet kaynağıyla.

**(a) İtki terimi $C_{I\leftarrow B}T_B$.** $T_B$ gövdede tanımlı (motor gövdeye vidalı), ama Newton'un yasası atalet çerçevesinde geçerli.

> **Somut örnek:** Roket 90° yatıksa ($q_i$'de olduğu gibi), gövdenin "ileri" yönü ($z_B$) dünyanın "yukarı" yönü değil **"yana"** yönüdür. Motor gövdeye göre tam düz ateşlese bile ($\delta^e=0$), bu itki dünyaya göre **yatay** bir kuvvettir. $C_{I\leftarrow B}$ olmadan roket yanlış yöne fırlar.

**(b) Aerodinamik $A_B$.** Bkz. §12.8.

**(c) Yerçekimi $g_I = (0,0,-g_0)$.** Bu terim **kütleye bölünmüyor** — $1/m$ çarpanının dışında. Çünkü yerçekimi ivmesi kütleden bağımsızdır (Galileo: tüy ve çekiç havasız ortamda aynı hızla düşer). $F=mg$ kuvvet, $a=g$ ivme — kütle sadeleşir. Ayrıca $g_I$ zaten I'da tanımlı, dönüşüme gerek yok.

```python
f = f.at[4:7].set( ((1 / x[0]) * (jnp.dot(CIB, (T_v + A_B)))) + g_I )
```

`(1/x[0])` yalnızca itki+aero toplamını çarpıyor, `g_I` dışarıda — fiziği birebir yansıtıyor.

#### 12.7.4 Tutum: quaternion kinematiği

$$\dot q_{B\leftarrow I} = \frac{1}{2}\,\Omega(\omega_B)\, q_{B\leftarrow I}$$

Türetim ve tekillik tartışması §12.3'te. Operatörün açık formu:

$$\Omega(\omega_B) = \begin{bmatrix} 0 & -p & -q & -r \\ p & 0 & r & -q \\ q & -r & 0 & p \\ r & q & -p & 0\end{bmatrix}, \qquad \omega_B = (p,q,r)$$

Kodda dört ayrı satıra açılmış:

```python
ox = omega(x[11:14])
f = f.at[7].set( 1/2 * (ox[0,0]*x[7] + ox[0,1]*x[8] + ox[0,2]*x[9] + ox[0,3]*x[10]))
f = f.at[8].set( 1/2 * (ox[1,0]*x[7] + ox[1,1]*x[8] + ox[1,2]*x[9] + ox[1,3]*x[10]))
f = f.at[9].set( 1/2 * (ox[2,0]*x[7] + ox[2,1]*x[8] + ox[2,2]*x[9] + ox[2,3]*x[10]))
f = f.at[10].set(1/2 * (ox[3,0]*x[7] + ox[3,1]*x[8] + ox[3,2]*x[9] + ox[3,3]*x[10]))
```

Bu tam olarak $\Omega(\omega_B)\,q$ matris-vektör çarpımı — sadece elle açılmış. `ox @ x[7:11]` de yazılabilirdi; elle açmak JAX'in küçük sabit-boyutlu matrisler için derleme grafiğinde küçük bir verimlilik farkı yaratabiliyor.

#### 12.7.5 Açısal hız: Euler'in dönme denklemi

$$\dot\omega_B = J_B^{-1}\Big(\underbrace{[r_{cm,B}\times]\,T_B}_{\text{itki momenti}} + \underbrace{[r_{cp,B}\times]\,A_B}_{\text{aero momenti}} - \underbrace{\omega_B \times (J_B\,\omega_B)}_{\text{jiroskopik terim}}\Big)$$

**(a) İtki momenti.**

> ⚠️ **Yön tanımı — dikkat.** Makalenin kendi ifadesi: *"the vector from the vehicle's center of mass **to** the engine gimbal hinge point"*. Yani $r_{cm,B}$: **kütle merkezinden gimbal menteşesine.** Alt indeks "cm", vektörün *nereden başladığını* gösteriyor. Aynı kural $r_{cp,B}$ için: kütle merkezinden basınç merkezine.
>
> **Neden bu yön zorunlu:** Tork $M = r\times F$ formülünde $r$, torkun hesaplandığı noktadan (kütle merkezi — Euler denklemi CM etrafındaki dönmeyi yazıyor) kuvvetin uygulandığı noktaya (gimbal) doğru olmalıdır. Tersi alınırsa tork işareti ters çıkar.

$r_{cm,B} = (0,0,-14)$ m. Motor roketin kuyruğunda, kütle merkezinin altında; CM'den başlayıp motora gitmek $z_B$'de negatif yönde 14 m.

**Moment nasıl doğuyor:** Gimbal açısı sıfırsa ($\delta^e=0$) itki tam gövde eksenine paralel — moment kolu ile kuvvet aynı doğrultuda, çapraz çarpım sıfır, **moment yok**. Gimbal açılırsa itki eksenden kayar, moment doğar.

> **Roketin nasıl döndürüldüğünün tam mekanizması budur:** pervane veya kanatçık değil, doğrudan itkinin yönünü kaydırmak.

**(b) Aerodinamik momenti.** Aynı mantık, farklı moment kolu: $r_{cp,B}=(0,0,3)$ m — **basınç merkezi**, hava direncinin etkin uygulama noktası.

> **Neden CP ≠ CM:** Kütle merkezi kütlenin nerede yoğunlaştığını, basınç merkezi hava basıncının net etkisinin nerede toplandığını gösterir — biri kütle dağılımına, diğeri geometriye bağlıdır. Aradaki fark roketin **aerodinamik stabilitesini** belirler (CP, CM'in gerisindeyse ok gibi kendini doğrultur — weathervaning).

**(c) Jiroskopik terim — dış kuvvet yok, yine de moment var.**

Bu terim hiçbir dış kuvvetten gelmiyor; yalnızca roketin **kendi dönüşünden** kaynaklanıyor.

**Matematiksel köken.** Euler denklemi açısal momentum korunumundan ($\dot L|_I = M$) türetilir. Ama $\omega_B$ **B çerçevesinde** takip ediliyor — dönen bir çerçevede. Dönen çerçevede türev alırken ekstra terim çıkar (Coriolis ile aynı aile):

$$\frac{dL}{dt}\bigg|_I = \frac{dL}{dt}\bigg|_B + \omega_B \times L$$

$L = J_B\omega_B$ konup $J_B\dot\omega_B$'ye çözülünce jiroskopik terim ortaya çıkar.

**Bisiklet tekerleği analojisi:** Dönen bir tekerleği elinizde tutup eksenini çevirmeye çalışın — beklemediğiniz bir yönde direnç hissedersiniz (jiroskopik presesyon). Yeni bir kuvvet uygulamadınız; tekerlek kendi dönüşü yüzünden böyle davranıyor.

**Roket için somut:** Roket $x$ ekseni etrafında hızlı dönüyorsa ($\omega_B=(p,0,0)$, $p$ büyük) ve gimbal ile $y$ ekseninde moment uygulanırsa, jiroskopik terim bu momentin bir kısmını **$z$ eksenine sızdırır** — roket komut verilen eksende değil, ona dik bir eksende de tepki verir. Uçuş kontrolcüsü tasarımında ihmal edilirse kararsızlığa yol açabilecek bir çapraz-eksen etkileşimi.

**Neden nonkonveks:** $\omega_B \times (J_B\omega_B)$ ifadesi $\omega_B$'de **kuadratik** ($\omega$ çarpı $\omega$). Bkz. §12.10.

```python
f = f.at[11:14].set(
    jnp.dot(J_B_inv_m,
        jnp.dot(skew(r_cm), T_v[0:3])
      + jnp.dot(skew(r_cp), A_B)
      - jnp.dot(skew(x[11:14]), (J_B_m @ x[11:14]))
    )
)
```

> Bu modelin roll momenti üretememesi hakkında: **§7.9**.

#### 12.7.6 Beş satırın özeti

| # | Denklem | Fiziksel köken | Konveks mi |
|---|---|---|---|
| 1 | $\dot m = -\alpha_{\dot m}T$ | Roket denklemi | Lineer ✓ |
| 2 | $\dot r_I = v_I$ | Tanım | Lineer ✓ |
| 3 | $\dot v_I = \tfrac{1}{m}C_{I\leftarrow B}(T_B+A_B)+g_I$ | Newton II | **Nonkonveks** |
| 4 | $\dot q = \tfrac12\Omega(\omega_B)q$ | Kinematik | **Nonkonveks** |
| 5 | $\dot\omega_B = J_B^{-1}(\ldots-\omega\times J_B\omega)$ | Euler dönme denklemi | **Nonkonveks** |

### 12.8 Aerodinamik kuvvet

$$A_B(t) = -\frac{1}{2}\,\rho\,\|v_I(t)\|_2\,S_A\,C_A\,C_{B\leftarrow I}(t)\,v_I(t)$$

#### 12.8.1 Klasik formülden geliş

Havacılıkta standart sürükleme denklemi $F_{drag} = \tfrac12\rho v^2 S C_D$. Makalenin formülü bunun **vektörel ve genelleştirilmiş** hali.

#### 12.8.2 Neden $\|v_I\|\cdot v_I$ (hız-kare **ve** yön)

Hız formülde iki kere geçiyor: $\|v_I\|$ (skaler büyüklük) ve $C_{B\leftarrow I}v_I$ (vektör yön). Çarpımları etkin olarak $v^2$ verir ama **yönü korur.**

**Neden düz $v^2$ değil:** Sürükleme her zaman hıza **zıt** olmalı. Bileşenler ayrı ayrı karelenseydi ($v_x^2$ vb.) negatif bileşenler pozitife dönerdi ve yön bilgisi kaybolurdu. $\|v\|\cdot v$ yapısı büyüklüğü kare gibi büyütürken **işareti $v$'den miras alır**; baştaki eksiyle birlikte sürükleme hep doğru yönde çıkar.

```python
v_norm = (x[4]**2 + x[5]**2 + x[6]**2 + 1e-8)**(0.5)
A_B = - 0.5 * params['rho_air'] * v_norm * params['S_area'] * (params['C_aero'] @ (CBI @ x[4:7]))
```

> **Neden $+10^{-8}$:** $\|v\|=\sqrt{v_x^2+v_y^2+v_z^2}$ fonksiyonunun türevi $v=0$'da tanımsızdır. Bu küçük sabit, karekök içini asla tam sıfır yapmayarak Jacobian'ın ($\partial A_B/\partial v$) patlamasını önler — otomatik türev için güvenlik payı. Roket durağan hızdan geçerse bu olmadan `jacfwd` NaN üretebilirdi.

#### 12.8.3 Neden hız önce gövde çerçevesine çevriliyor

Formülde $C_{B\leftarrow I}v_I$ var: hız I'dan B'ye çevriliyor, kuvvet B'de hesaplanıyor, sonra hız denkleminde tekrar $C_{I\leftarrow B}$ ile I'ya çevriliyor.

**Neden bu gidiş-dönüş:** $C_A$ matrisi **gövdeye göre** tanımlı — rüzgâr tünelinde ölçümler modele göre yapılır, dünyaya göre değil. Hesap B'de yapılmalı; ama Newton'un yasasına sokmadan önce I'ya dönmek gerekir.

#### 12.8.4 $C_A$ matrisi neden köşegen

$$C_A = \mathrm{diag}(0.4068,\; 0.4068,\; 0.0522)$$

Roket gövde eksenlerinde simetrik bir silindir. $x_B, y_B$ katsayıları **eşit** (0.4068) — hangi yandan rüzgâr gelirse gelsin yanal direnç aynı. $z_B$ (boylamasına) çok daha küçük (0.0522) — ince uzun bir cisim boyuna doğrultuda çok az direnç görür; bir kalemi düz tutup sallamak, yan tutup sallamaktan kolaydır.

**Köşegen olması** çapraz terimlerin sıfır olduğunu söyler: $x_B$ yönündeki hız, $y_B$ veya $z_B$'de kuvvet üretmiyor. Bu bir basitleştirme (gerçekte kanatçık asimetrisi gibi küçük çapraz etkiler olabilir) ama silindirik gövde için makul.

### 12.9 Operatörler: `skew` ve `omega`

#### 12.9.1 `skew` — çapraz çarpımı matrise çevirmek

```python
def skew(v):
    return jnp.array([
        [0,    -v[2],  v[1]],
        [v[2],     0, -v[0]],
        [-v[1], v[0],     0]
    ])
```

**Neden gerekli:** $r\times T$'yi JAX'in otomatik türev sistemine "matris çarpımı" olarak sunmak, özel bir operasyon olarak sunmaktan daha standart. `skew(r) @ T` cebirsel olarak $r\times T$ ile **birebir aynı** — yaklaşıklık değil, kesin eşitlik.

**Doğrulama:**

$$r \times T = \begin{pmatrix} r_2T_3 - r_3T_2 \\ r_3T_1 - r_1T_3 \\ r_1T_2 - r_2T_1\end{pmatrix}, \qquad
\begin{bmatrix}0 & -r_3 & r_2 \\ r_3 & 0 & -r_1 \\ -r_2 & r_1 & 0\end{bmatrix}\begin{pmatrix}T_1\\T_2\\T_3\end{pmatrix} = \begin{pmatrix}-r_3T_2+r_2T_3 \\ r_3T_1-r_1T_3 \\ -r_2T_1+r_1T_2\end{pmatrix}$$

Aynı sonuç ✓

**Çarpık simetrik:** Devriğini alın — köşegenin üstü ve altı işaret değiştirir, $M^\top = -M$. Bu, §12.3.5'te $\Omega$ için kullandığımız **aynı yapı**; çapraz çarpımın kendisiyle her zaman dik olması ($r\times T \perp r$) buradan gelir.

> ⭐ **Kalıp:** $\Omega(\omega_B)$ (quaternion kinematiği) ve `skew(r)` (moment hesabı) matematiksel olarak **aynı aile** — ikisi de çarpık simetrik, ikisi de bir çapraz-çarpım-benzeri işlemi matrise gömüyor. Rotasyon geometrisinde tekrar tekrar karşınıza çıkan bir yapı.

#### 12.9.2 `omega` — makaleyle birebir eşleşiyor

```python
def omega(w):
    return jnp.array([
        [0,    -w[0], -w[1], -w[2]],
        [w[0],     0,  w[2], -w[1]],
        [w[1], -w[2],     0,  w[0]],
        [w[2],  w[1], -w[0],     0],
    ])
```

Makalenin $\Omega(\xi)$ tanımıyla birebir aynı (`w[0]`→$\xi_1$ vb.). İşaret farkı **yok**; §12.3.5'teki türetim buradan doğrudan doğrulanıyor.

### 12.10 Nonkonvekslik haritası

SCP'nin her iterasyonda **tam olarak neyi** lineerleştirdiğinin haritası:

| Terim | Nerede | Nonlineerlik türü |
|---|---|---|
| $C_{B\leftarrow I}(q),\; C_{I\leftarrow B}(q)$ | Hız denklemi, aero | Quaternion'da **kuadratik** ($q_iq_j$ çarpımları) |
| $\tfrac{1}{m}\cdot(\ldots)$ | Hız denklemi | **Bilineer** — $m$, $q$ ve $u$ birbiriyle çarpılıyor |
| $\|v_I\|\cdot v_I$ | Aerodinamik | Norm × kendisi |
| $\Omega(\omega_B)\,q$ | Tutum kinematiği | **Bilineer** ($\omega$ ile $q$ çarpımı) |
| $\omega_B \times (J_B\omega_B)$ | Açısal hız | $\omega$'da **kuadratik** |

#### 12.10.1 $C(q)$ neden kuadratik — ve "$SO(3)$ doğrusal değil" ne demek

Matrisin her elemanı quaternion bileşenlerinin ya karesi ($q_3^2$) ya çarpımı ($q_2q_3$) — hiçbiri birinci dereceden değil.

**Derin sebep:** $SO(3)$ (rotasyon matrisleri kümesi) matris çarpımı altında kapalıdır ama **toplama altında kapalı değildir** — iki rotasyon matrisini eleman-eleman toplarsanız sonuç genelde rotasyon matrisi olmaz (satırlar birim uzunlukta ve dik kalmaz).

**Saat analojisi:** "Saat 3 + saat 5" anlamlı bir toplama değildir. Saat pozisyonları dairesel bir yapıda yaşar, düz bir vektör uzayında değil. $SO(3)$ de böyle **eğri bir yüzeyde (manifold)** yaşar. Quaternion bileşenlerindeki kuadratik terimler tam olarak bu eğriliği kodlar; lineer bir formülle yakalanamaz.

#### 12.10.2 $1/m$ — sık yapılan bir hatayı düzeltelim

> ⚠️ **"$1/m$ konveks değildir" demek yanlıştır.** $f(m)=1/m$ için $f''(m) = 2/m^3 > 0$ ($m>0$) — fonksiyon **konvekstir**.

**Asıl sorun $1/m$'nin kendisi değil, çarpım yapısıdır:**

$$\dot v_I = \underbrace{\frac{1}{m}}_{\text{durum}} \cdot \underbrace{C_{I\leftarrow B}(q)}_{\text{durum}} \cdot \underbrace{(T_B(u) + A_B(x))}_{\text{durum + kontrol}}$$

Üç ayrı değişken grubu birbiriyle çarpılıyor — **bilineer** (çok-lineer) terim.

**Hessian testi.** $g(m,T) = T/m$ için:

$$H = \begin{bmatrix} 2T/m^3 & -1/m^2 \\ -1/m^2 & 0 \end{bmatrix}, \qquad \det H = -\frac{1}{m^4} < 0$$

Negatif determinant → Hessian pozitif tanımlı değil → **eyer noktası (saddle)** yapısı. §5.2'deki "eyer şeklindeki kuadratik kısıt konveks değildir" örneğiyle aynı sınıf.

**Dikdörtgen analojisi:** Alan $= u \times g$. Uzunluk ve genişlik ayrı ayrı gayet uysal, ama **çarpımları** $u$-$g$ düzleminde eyer şeklinde bir yüzey çizer. Sabit alanlı dikdörtgenlerin kümesi ($u\cdot g = 10$) bir **hiperbol** — konveks değil.

**Doğru genel ifade:**

> Dinamik $\dot x = F(x,u)$ bir **eşitlik kısıtıdır.** Bir eşitlik kısıtının tanımladığı küme, **ancak ve ancak $F$ afin (lineer + sabit) ise** konvekstir. $F$ içinde herhangi bir çarpım, kare veya bölme varsa, o eşitlik kısıtı genel olarak nonkonveks bir küme tanımlar. Mesele $F$'nin kendi konvekslik sınıfı değil, **doğrusal olup olmadığıdır.**

#### 12.10.3 Konveks kalanlar

| Terim | Neden konveks |
|---|---|
| $\dot m = -\alpha T$ | Lineer |
| $\dot r_I = v_I$ | Lineer |
| $g_I$ | Sabit |
| Tüm yol kısıtları (tilt, açısal hız, glideslope, gimbal) | Afin / konveks kuadratik / konik (§5.2) |

> **Sonuç:** Dinamiğin beş satırından **üçü** nonkonveks. SCP'nin işi tam olarak bunları her iterasyonda Jacobian alıp lineerleştirmek. Kısıtlar bu listede **hiç yok** — onlara dokunulmuyor (§4.1.1).

### 12.11 Kodla tam eşleşme — Bölüm II.A'nın tamamı

```python
def dynamics(x, u):
    CBI = CBI_fcn(x[7:11])              # C_{B<-I}(q)
    CIB = CBI.transpose()               # C_{I<-B} = C_{B<-I}^T

    J_B_inv_m = J_B_inv_pre / x[0]      # J_B^{-1}, kutleyle olcekli
    J_B_m     = J_B_pre * x[0]          # J_B, kutleyle olcekli

    # 1. Kutle
    f = f.at[0].set(- params['alpha_m'] * u[0])

    # 2. Konum
    f = f.at[1:4].set(x[4:7])

    # Aerodinamik (3 ve 5'te kullanilacak)
    v_norm = (x[4]**2 + x[5]**2 + x[6]**2 + 1e-8)**0.5
    A_B = -0.5 * params['rho_air'] * v_norm * params['S_area'] * (params['C_aero'] @ (CBI @ x[4:7]))

    # Itki vektoru (bkz. 12.4.1)
    T_v = jnp.array([u[0]*jnp.sin(u[1])*jnp.cos(u[2]),
                      u[0]*jnp.sin(u[1])*jnp.sin(u[2]),
                      u[0]*jnp.cos(u[1])])

    # 3. Hiz
    f = f.at[4:7].set(((1/x[0]) * jnp.dot(CIB, (T_v + A_B))) + g_I)

    # 4. Tutum (Omega @ q, 4 satira acilmis)
    ox = omega(x[11:14])
    f = f.at[7].set(0.5*(ox[0,0]*x[7]+ox[0,1]*x[8]+ox[0,2]*x[9]+ox[0,3]*x[10]))
    f = f.at[8].set(0.5*(ox[1,0]*x[7]+ox[1,1]*x[8]+ox[1,2]*x[9]+ox[1,3]*x[10]))
    f = f.at[9].set(0.5*(ox[2,0]*x[7]+ox[2,1]*x[8]+ox[2,2]*x[9]+ox[2,3]*x[10]))
    f = f.at[10].set(0.5*(ox[3,0]*x[7]+ox[3,1]*x[8]+ox[3,2]*x[9]+ox[3,3]*x[10]))

    # 5. Acisal hiz (itki momenti + aero momenti - jiroskopik)
    f = f.at[11:14].set(jnp.dot(J_B_inv_m,
          jnp.dot(skew(r_cm), T_v[0:3])
        + jnp.dot(skew(r_cp), A_B)
        - jnp.dot(skew(x[11:14]), (J_B_m @ x[11:14]))
    ))
```

### 12.12 Adım 3 — Tez entegrasyon analizi

#### 12.12.1 Durum/kontrol eşlemesi

Makalenin 14 boyutlu durum vektörü, PROJE_BAGLAMI §1'deki 6-DOF hedefiyle **yapısal olarak birebir örtüşüyor** — Wang & Song'un 3-DOF modelinde bu yoktu.

| Makale | Simulink modeli (beklenen) | Uyum |
|---|---|---|
| $m$ | Kütle durumu | ✓ |
| $r_I, v_I$ | Konum/hız blokları | ✓ |
| $q_{B\leftarrow I}$ | Quaternion (§4.2'de zaten seçilmiş) | ✓ |
| $\omega_B$ | Açısal hız | ✓ |
| $u=(T,\delta^e,\phi^e,\delta^b,\phi^b)$ | Thrust Vectoring subsystem çıkışı | **kısmi** — §12.12.2 |

- [ ] **Açık:** Simulink modelindeki durum vektörü sırası makalenin sırasıyla aynı mı? Model paylaşıldığında doğrulanacak.

#### 12.12.2 Gimbal parametrizasyonu uyuşmazlığı

PROJE_BAGLAMI §4.4: *"Thrust Vectoring subsystem: çift kademeli saturasyon — `Zeta_CMD` önce `max_noz_ang_rad`'ın ±%75'ine sınırlanır..."*

`Zeta_CMD` ismi **kartezyen** bir gimbal komutuna işaret ediyor ($\zeta_x, \zeta_y$), makale ise **küresel** parametrizasyon kullanıyor ($\delta^e, \phi^e$). Matematiksel olarak eşdeğer ama aynı değil:

$$\zeta_x = T\sin\delta^e\cos\phi^e, \qquad \zeta_y = T\sin\delta^e\sin\phi^e$$

| | Kartezyen | Küresel |
|---|---|---|
| Kısıt şekli | Kare/kutu | **Disk (dairesel)** |
| Fiziksel gerçekçilik | — | Nozzle genelde dairesel hareket eder |

> ⭐ **Açık karar noktası:** SCP küresel parametrizasyon kullanırsa, çıkan yörünge Simulink'in izin verdiğinden **farklı bir gimbal kısıt şekli** varsayıyor olabilir. Trajectory'yi Simulink'e beslerken gözden geçirilmeli.

#### 12.12.3 Atalet modeli genişletmesi

$$\text{Makale:}\quad J_B(t) = m(t)\cdot\mathrm{diag}([60,60,1.5]), \quad r_{cm,B} = \text{sabit}$$

$$\text{Sizin §4.2 kararınız:}\quad J_B(t) = J_B\big(m(t), r_{cm,B}(t)\big), \quad r_{cm,B}(t) = f(\text{yakıt tüketimi})$$

Dinamik fonksiyonda **yalnızca iki yeri** etkiler:

```python
J_B_m = J_B_pre * x[0]                  # -> J_B_fonksiyonu(x[0], r_cm(x[0]))
jnp.dot(skew(r_cm), T_v[0:3])           # -> r_cm artik x[0]'in fonksiyonu
```

> **Pratik kolaylık:** JAX otomatik türev kullandığı için (§6.3), $r_{cm}(m)$'yi $m$'nin fonksiyonu olarak yazmanız yeterli — Jacobian zinciri otomatik güncellenir. PROJE_BAGLAMI §6'daki "6-DOF'ta elle Jacobian riskli" endişesini doğrudan çözüyor.

#### 12.12.4 Hiyerarşik mimari — **A seçeneği (karar alındı)**

| Seçenek | Üst katman | Alt katman | Karar |
|---|---|---|---|
| **A** | Bu 6-DOF model, kaba ayrıklaştırma ($K$ küçük) | Aynı model, ince ayrıklaştırma + kayan ufuk ([62]) | ✅ **Seçildi** |
| B | Basitleştirilmiş 3-DOF planlayıcı | 6-DOF takipçi | Elendi (iki modelin tutarlılığını doğrulama yükü) |

> A'nın bedeli daha ağır hesaplama yüküdür; avantajı tek model, tek tutarlılık. **[62] incelendikten sonra tekrar değerlendirilecek.**

#### 12.12.5 Çoklu iniş noktası — bozucu sonrası site değişimi

**Hedef (kullanıcı kararı):** Roket tek bir noktayı körü körüne takip etmemeli; bozucu sonrası daha iyi bir site varsa oraya geçebilmeli. **Olmazsa olmaz değil — önce düz dikey iniş çalışsın.**

**Problemin doğası.** "Site 1 mi site 2 mi" **ayrık** bir karardır — arada bir şey yok. Bu, fizibil kümeyi **kopuk** hale getirir: iki ayrı vadi, arada köprü yok. SCP bir vadiye düşer ve çıkmaz; başlangıç tahmini site 1'i gösteriyorsa site 2'yi **asla keşfetmez** (§4.1.2, dağ analojisi; §3.4.2.0, yasak boşluk).

| Yaklaşım | Nasıl | Değerlendirme |
|---|---|---|
| **A. Her ikisini çöz, iyisini seç** | SCP'yi iki farklı başlangıç tahminiyle çalıştır, maliyetleri karşılaştır | ✅ **Pratik ve doğru.** 2 site = 2× hesap; iterasyon ~0.1 s olduğuna göre kabul edilebilir |
| B. Mixed-integer | İkili değişken + MIP | ❌ $2^K$ patlaması (§3.4.2.0) |
| C. Homotopy | Sürekli deformasyonla ayrık mantık — [41] Malyuta & Açıkmeşe | Zarif ama tez kapsamı için ağır |

> ⚠️ **D-GMSR bunu tek başına çözmez.** "Site 1 VEYA site 2" mantığını pürüzsüz, sound ve complete şekilde kodlayabilir — ama **pürüzsüzleştirme çok-tepeliliği (multi-modality) ortadan kaldırmaz.** D-GMSR locality & masking'i çözer (§4.11), kopuk fizibil kümede global optimumu bulmayı değil.

**Uçuş ortasında geçiş — MPC'ye özgü sorun.** Warm start (§6.12) sizi kilitler: her adımda bir önceki çözüm başlangıç tahmini olduğu için **hep aynı vadide kalırsınız.** MPC kendiliğinden asla site değiştirmez.

**Gereken mimari:**

```
DENETLEYICI KATMAN (cok yavas, ~1 s)
  Her N adimda: HER IKI siteyi de coz, maliyetleri karsilastir
  Karar: site degissin mi?
              |
UST KATMAN (yavas, ~1-2 s)
  Secilen siteye 6-DOF yorunge planla (bu makale)
              |
ALT KATMAN (hizli, ~0.1 s)
  Yorungeyi takip et (MPC, [62])
```

**İki kritik kontrol sorunu:**

| Sorun | Ne olur | Çözüm |
|---|---|---|
| **Chattering** | Maliyetler yakınsa (%1 fark) ölçüm gürültüsü her adımda kararı ters çevirir; roket iki site arasında salınır | **Histerezis:** sadece belirli marjla (ör. %5) daha iyiyse geç; geçişten sonra **minimum bekleme süresi** (ör. 3 s) |
| **Point of no return** | Belirli irtifanın altında geçiş fiziksel olarak imkânsız; fizibil olmayan probleme çözüm aranır | **Commit altitude:** eşiği elle seçmek yerine **fizibilite testiyle** bul — her iki siteye çözüm fizibil olduğu sürece karar açık, biri fizibil olmaktan çıkınca kilitle |

**Literatür bağlantısı.** §3.6'daki **G-FOLD**'un açılımına dikkat: **G**uidance for **F**uel **O**ptimal **L**arge **D**ivert. "Divert" = sapma, rota değiştirme. Xombie uçuş testleri ([25],[26]) 500 m ve 750 m'lik **sapma manevralarıdır.** Mars 2020'nin Terrain Relative Navigation sistemi de tehlikeli bölge tespitinde daha güvenli noktaya sapma yeteneğine sahipti.

> ⭐ **Tez katkısı:** "Sürekli-zaman kısıt garantili 6-DOF MPC + çoklu iniş noktası arasında bozucu-tetiklemeli geçiş" birleşimi literatürde hazır bir paket olarak yok. G-FOLD 3-DOF ve açık-döngü; bu makale 6-DOF ama tek site ve MPC değil.

#### 12.12.6 Quaternion kararının teorik doğrulanması

§12.3.5'teki $\frac{d}{dt}\|q\|^2 = 0$ türetimi, PROJE_BAGLAMI §4.2'deki normalizasyon kararını destekliyor.

> **Tez metninde kullanılabilecek cümle:** *"Quaternion kinematiği analitik olarak norm-koruyucudur; normalizasyon adımı yalnızca sayısal kararlılık içindir ve ek bir fiziksel varsayım getirmez."*

#### 12.12.7 Eylem maddeleri

- [ ] Simulink durum vektörü sırasını makaleyle karşılaştır (§12.12.1)
- [ ] Gimbal parametrizasyonu: kartezyen mi küresel mi (§12.12.2)
- [ ] $r_{cm}(m)$ fonksiyonunu tanımlayıp koda ekle — iki satır (§12.12.3)
- [ ] [62] okunduktan sonra mimari A kararını tekrar değerlendir (§12.12.4)
- [ ] Çoklu site: "her ikisini çöz + histerezis + commit altitude" — düz iniş çalıştıktan **sonra** (§12.12.5)
- [x] Roll ihmal kararı — açık varsayım olarak tez metnine yazılacak (§7.9)
- [x] Tek nozzle kararı — makalenin yapısı alınacak

---

## 13. Bölüm II.B — General State and Control Constraints

> **Durum:** Adım 1–4 tamamlandı. Dört durum + dört kontrol kısıtı; $g_x(x)\le0_{4\times1}$, $g_u(u)\le0_{4\times1}$ olarak tek vektörde toplanıyor.

### 13.1 Neden "≤ 0" kalıbı — dört kısıtı tek sembole sıkıştırmak

Kuru kütle kısıtının doğal yönü ($m_{dry}\le m$) tek başına ters; diğer üçü ($\theta,\omega,\gamma$) zaten "≤" formunda. Hepsini $g_x(x)\le0$ diye tek vektörde yazabilmek için kuru kütle de çevrilir:

$$m_{dry}\le m(t) \;\Longleftrightarrow\; -m(t)\le -m_{dry}$$

Bu salt bir gösterim işlemi — fiziksel anlam değişmiyor. Asıl gerekçesi: §5.4'teki CTCS ceza fonksiyonu $q_c(0,g)=\max(0,g)^2$, kısıtın "≤0" formunda olduğunu varsayıyor (ihlalse pozitif, değilse 0) — dört kısıt bu kalıba önceden sokulunca **tek formül, istisnasız** uygulanabiliyor.

> Kodda bu dönüşüm **yapılmamış** (`X[0,:] >= m_dry`, doğal yön) — çözücü yönden bağımsız çalıştığı için sorun değil; "≤0" kalıbı yalnızca makalenin notasyonu ve CTCS formülü için gerekliydi.

### 13.2 Dört durum kısıtı

| # | Kısıt | Sınıf (§5.2) | Not |
|---|---|---|---|
| 1 | $m_{dry}\le m(t)$ | Afin | Düz duvar |
| 2 | $\cos\theta_{max}\le1-2(q_2^2+q_3^2)$ | Konveks kuadratik | §13.2.1 |
| 3 | $\|\omega_B\|_2\le\omega_{max}$ | Konik (SOC) | Genelde aktif değil ($\omega_{max}=90°/s$ cömert) |
| 4 | $\tan(\gamma_{max})\|[e_1e_2]^\top r_I\|_2\le e_3^\top r_I$ | Konik (SOC) | §13.2.2, bkz. §7.2 errata |

#### 13.2.1 Tilt kısıtının türetimi ve "yaw'a kör" doğrulaması

$C_{B\leftarrow I}$'nin (3,3) elemanından: $\cos\theta=1-2(q_2^2+q_3^2)$. $\theta\le\theta_{max}\Leftrightarrow\cos\theta\ge\cos\theta_{max}$ (kosinüs azalan), düzenlenince yarım-açı özdeşliğiyle:

$$\sqrt{q_2^2+q_3^2}\;\le\;\sin\!\Big(\frac{\theta_{max}}{2}\Big)$$

$(q_2,q_3)$ düzleminde yarıçapı $\sin(\theta_{max}/2)$ olan bir **disk** — konveks kuadratik ailenin somut örneği. Kod: `sqrt((1-cos(theta_max))/2)`.

**Yaw'a kör olduğunun kanıtı — tilt ve spin'i ayrı quaternion'lara ayırıp çarpma:**

$$q_{tilt}=\big(\cos\tfrac\theta2,0,\sin\tfrac\theta2,0\big),\qquad q_{spin}=\big(\cos\tfrac\psi2,0,0,\sin\tfrac\psi2\big)$$

Hamilton çarpımıyla birleştirince $q_2=\sin\tfrac\theta2\sin\tfrac\psi2$, $q_3=\sin\tfrac\theta2\cos\tfrac\psi2$, dolayısıyla:

$$q_2^2+q_3^2=\sin^2\tfrac\theta2\big(\sin^2\tfrac\psi2+\cos^2\tfrac\psi2\big)=\sin^2\tfrac\theta2$$

$\psi$ (yaw) $\sin^2+\cos^2=1$ özdeşliğiyle **tamamen iptal oluyor.** $(q_2,q_3)$ düzleminde nokta, yaw değiştikçe sabit yarıçaplı bir çember üzerinde gezer — merkeze uzaklığı (yani kısıtın ölçtüğü şey) hep aynı. Simetrik doğrulama: $q_2^2+q_4^2=\sin^2\tfrac\psi2$ — bu sefer $\theta$ iptal olup **saf yaw** kalıyor. Hangi çiftin toplandığı (($q_2,q_3$) vs ($q_2,q_4$)) hangi bilginin (tilt vs yaw) yakalandığını belirliyor.

#### 13.2.2 Glideslope türetimi (bkz. §7.2 için konvansiyon uyuşmazlığı)

$d=\|[e_1e_2]^\top r_I\|_2$ (yatay mesafe), $h=e_3^\top r_I$ (irtifa). $\gamma$ **yataydan** ölçülürse $\tan\gamma=h/d$, ve $\gamma\le\gamma_{max}\Rightarrow h\ge\tan(\gamma_{max})d$ — makalenin yazdığı forma tek adımda ulaşılır. Bu, koninin her sabit $h$'de dairesel bir kesiti olduğunu gösterir ($x^2+y^2\le(h\tan\gamma_{max})^2$) — üçüncü konveks aile (SOC).

$e_3^\top r_I$ ve $[e_1e_2]^\top r_I$ notasyonu: $e_i$ standart birim vektörler; $e_3^\top r_I=(0,0,1)\cdot(x,y,z)=z$ (sadece $z$'yi geçiren süzgeç), $[e_1e_2]^\top r_I=(x,y)$ (sadece $x,y$'yi geçiren süzgeç, $z$ atılır). Matris çarpımı olarak yazılması, çözücüye özel bir "seçme" fonksiyonu tanımlamadan standart matris-vektör çarpımıyla verilebilmesi içindir.

### 13.3 Dört kontrol kısıtı — ikisi anlamsız (vacuous)

$$\|\delta^e\|_1\le\delta^e_{max},\;\; \|\phi^e\|_1\le\phi^e_{max},\;\; \|\delta^b\|_1\le\delta^b_{max},\;\; \|\phi^b\|_1\le\phi^b_{max}$$

Kodda sadece **ikisi** uygulanmış:

```python
-delta_engine_max <= U[1,:] <= delta_engine_max      # delta^e
-delta_boresight_max <= U[3,:] <= delta_boresight_max # delta^b
```

$\phi^e,\phi^b$ (azimut) **hiç yok** — hata değil, bilinçli gözden çıkarma.

**Neden:** $\phi_{max}=180°$ demek $-180°\le\phi\le180°$, yani **tam bir tur** — çemberin her noktası. $\phi$ periyodiktir: $\phi=250°$ ile $\phi=-110°$ ($250-360$) **fiziksel olarak aynı yön.** Dinamiğe giren $\sin\phi,\cos\phi$ zaten periyodik, hangi aralıkta tutulursa tutulsun gerçek yön değişmez. $\delta$ ise gimbal mafsalının **gerçek mekanik limiti** (eksenden ne kadar saptığı) — bu yüzden aktif, $\phi$ (hangi yöne saptığı) ise dönel-simetrik mekanizmada sınırlanacak bir şey değil.

| | $\delta$ (sapma) | $\phi$ (azimut) |
|---|---|---|
| Ölçtüğü | Eksenden ne kadar | Hangi yöne |
| Fiziksel limit | Var (mafsal) | Yok (simetrik) |
| Kısıt | Gerçek/aktif | Vacuous — eklense de sonucu değiştirmez |

> **Tez için:** $\phi$ kısıtlarını eklemenize gerek yok; eklerseniz zararı olmaz (her zaman otomatik sağlanır), eklemezseniz kayıp yok.

---

## 14. Bölüm II.C — Boundary Conditions

> **Durum:** Adım 1, 3, 4 tamamlandı (kısa bölüm, ayrı denklem-parçalama adımı gerektirmedi).

### 14.1 On sınır koşulu, tek asimetri: kütle

$$m(0)=m_i,\;\; \boxed{m(t_f)\ge m_{dry}},\;\; r_I(0)=r_i,\;r_I(t_f)=r_f,\;\; v_I(0)=v_i,\;v_I(t_f)=v_f$$
$$q_{B\leftarrow I}(0)=q_{B\leftarrow I\,i},\;\;q_{B\leftarrow I}(t_f)=q_{B\leftarrow I\,f},\;\; \omega_B(0)=\omega_{B\,i},\;\;\omega_B(t_f)=\omega_{B\,f}$$

**Kütlenin sonu neden eşitsizlik, diğerleri eşitlik:** $r_f,v_f,q_f,\omega_{Bf}$ tasarım gereksinimlerinden önceden bilinir (iniş noktası, hız, tutum hedefi). Ama $m(t_f)$ **çözümün sonucudur** — ne kadar yakıt harcandığı, önceden sabitlenemez; sabitlenirse "en az yakıtla in" optimizasyonunun anlamı kalmaz. Bu, §13.2'deki yol kısıtı $m_{dry}\le m(t)$'nin $t=t_f$ özel hali — makale sınır koşullarını (Eq. 3f, $X$ kümesi) yol kısıtlarından (Eq. 3c) yapısal olarak ayrı tutuyor.

### 14.2 Notasyon netleştirmesi: $q_{B\leftarrow I\,i}$ neresi $i$, neresi $I$

$q_{B\leftarrow I\,i}$'de **iki katmanlı** alt indeks var: $B\leftarrow I$ (hangi çerçeveden hangisine — quaternion'un kimliği) ve $i$ (hangi zaman anı — $t=0$). $I_i,I_f$ diye ayrı atalet çerçeveleri **yoktur**; $I$ tanım gereği sabit. $i/f$ etiketi tüm $q_{B\leftarrow I}(t)$ fonksiyonuna, $t=0$ ve $t=t_f$ anlarındaki değerini isimlendirmek için ekleniyor — $r_I(0)=r_i$'de $I$'nin $i/f$ ile tamamen değişmesinden farklı olarak, quaternion'da $B\leftarrow I$ korunmak zorunda olduğu için etiket yanına eklenip görsel çakışma yaratıyor.

### 14.3 Quaternion sınır koşulu neden "kolay" — dinamikle tezat

$q_{B\leftarrow I}(t_f)=q_f$ dört ayrı **afin eşitlik** ($q_1=1,q_2=0,q_3=0,q_4=0$) — 4 boyutta **tek nokta**, konvekslik testini otomatik geçer (içinde iki nokta yok). Bu, §12.10.1'de gördüğümüz $C(q)$'nun kuadratik/nonkonveks oluşuyla **tezat**: nonkonveks olan quaternion'un **zamanla nasıl değiştiği** (dinamik), belirli bir anda **hangi değeri aldığını sabitlemek** (sınır koşulu) değil.

> Yol analojisi: arabanın gideceği yol eğri/zorlu olabilir (dinamik — nonkonveks), ama "varış adresi tam şurası" demek yolun şeklinden bağımsız basit bir hedef (sınır koşulu — konveks).

### 14.4 Sayısal değerlerin okunması

| Değişken | Başlangıç | Bitiş | Yorum |
|---|---|---|---|
| $m$ | 100 000 kg | $\ge85\,000$ kg | 15 t yakıt bandı, tam tüketim serbest |
| $r_I$ | $(200,200,500)$ m | $(0,0,0)$ m | İniş noktası orijin |
| $v_I$ | $(0,0,-50)$ m/s | $(0,0,-5)$ m/s | **Sıfır değil** — §14.4.1 |
| $q_{B\leftarrow I}$ | $(\tfrac{\sqrt2}2,\tfrac{\sqrt2}2,0,0)$ | $(1,0,0,0)$ | 90° yatıktan dikeye |
| $\omega_B$ | $(0,0,0)$ | $(0,0,0)$ | Segment öncesi zaten stabilize |

#### 14.4.1 $v_f=(0,0,-5)$ neden tam sıfır değil

İki olası gerekçe: **(a)** tam $v=0$'a inmek roketi yere değmeden hemen önce asılı bırakır — ilerleme kaydetmeden sadece yerçekimini dengelemek için yakıt yakmak (§8.2'deki gravity-loss tartışmasıyla aynı mekanizma); **(b)** gerçek iniş takımları (Apollo LM dahil) küçük bir çarpma hızını yutacak şekilde tasarlanır, tam sıfır hız hem gereksiz hem pratikte asimptotik (sonsuz zaman ister). **Tez için mühendislik dengesi:** küçük $|v_f|$ konforlu-ama-pahalı, büyük $|v_f|$ ucuz-ama-yapısal-risk.

### 14.5 $X$ kümesi aslında iki nokta gibi

Makale $X$'i "kapalı konveks küme" diye tanımlıyor ama somut hali $\{x(0)\}\times\{x(t_f):r_I=r_f,v_I=v_f,q=q_f,\omega_B=\omega_{Bf},m\ge m_{dry}\}$ — başlangıç tek sabit nokta, bitiş de neredeyse tek nokta (kütle yönünde gevşetilmiş bir ışın). Gerçek bir "çok noktalı bölge" (örn. iniş alanı serbest) yok.

> **§12.12.5'teki çoklu-site fikrine bağlantı:** "İniş A veya B olabilir" istenirse, $X$'in $r_I(t_f)=r_f$ kısmı **iki ayrı sabit noktadan biri** için ayrı ayrı çözülmeli — önerdiğimiz "iki problemi çöz, iyisini seç" yaklaşımı, $X$ kümesi düzeyinde $r_f$'yi değiştirip tekrar çözmeye denk geliyor.
>
> **Literatür alternatifi:** [14] (Blackmore, Açıkmeşe & Scharf — minimum-landing-error) $r_f$'yi serbest bırakıp sapmayı maliyete ekliyor ("tam oraya inemiyorsan en az sapmayla in"). Bu makale sıkı eşitliği seçmiş — §8.2'deki Case D ile örtüşüyor.

### 14.6 Kod eşleşmesi

```python
vehicle_cons += [
    X[:, 0]     == x_init,          # 15 durumun HEPSI, t=0
    X[1:-1, -1] == x_final[1:-1],   # indeks 1..13 (kutle VE y HARIC), t=tf
]
```

`1:-1` dilimlemesi tam olarak **kütle** ($x_0$) ve **CTCS sayacı $y$** ($x_{14}$) dışındaki her şeyi sabitliyor — makalenin $m(t_f)$'i muaf tutmasıyla birebir örtüşüyor. $y$'nin de hariç tutulması tutarlı: $y$'nin kendi sınır koşulu ayrıca ele alınıyor ($y(t_f)-y(0)\le\epsilon_{LICQ}$, §5.4.6).


## Değişiklik geçmişi

| Tarih | Ne eklendi |
|---|---|
| — | **Bölüm II.B ve II.C — tüm adımlar.** Durum/kontrol kısıtları (tilt yaw-körlüğü kanıtı, glideslope konvansiyon çözümlemesi, azimut vacuous kanıtı), sınır koşulları (kütle asimetrisi, quaternion notasyon netleştirmesi, X kümesi ve çoklu-site bağlantısı). §7.2 errata genişletildi. |
| — | **Bölüm II.A — Adım 1 (2/3, 3/3), Adım 3, Adım 4.** Dinamiğin beş satırı, aerodinamik, operatörler, nonkonvekslik haritası, tez entegrasyon analizi (mimari A, çoklu site, roll bulgusu §7.9). |
| — | **Bölüm II.A — Adım 1, Bölüm 1/3.** Referans çerçeveleri, durum/kontrol vektörleri, Euler tekilliği (türetimle), gimbal parametrizasyonu. 2 inline SVG. |
| — | **Bölüm I (Introduction) — Adım 1 tamamlandı.** Ek olarak: kod deposu analizi (§6), errata (§7), tez entegrasyon planı (§8), terimler sözlüğü (§10), referans haritası (§11). 3 inline SVG. |

> **Sonraki:** Bölüm II.B — General state and control constraints. Ardından II.C (sınır koşulları) ve II.D (compound STC'ler).
