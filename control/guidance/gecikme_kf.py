"""
gecikme_kf.py — GÖRÜNTÜ GECİKMESİNE UYUMLU DURUM KESTİRİMİ (Ö-KF)

Kaynak makale
─────────────
  B. M. Nguyen, W. Ohnishi, Y. Wang, H. Fujimoto, Y. Hori (Tokyo Üniv.) +
  K. Ito ve ark. (Hitachi), "Dual Rate Kalman Filter Considering Delayed
  Measurement and Its Application in Visual Servo", AMC2014, Yokohama.

MAKALEDEN ALINAN (bize uyan kısımlar)
──────────────────────────────────────
  1) ⭐ ANA FİKİR — gecikmeli ölçümü DOĞRU ÇAĞLA eşle.
     Kamera karesi τ saniye eskiyse, onu "şimdiki" tahminle karşılaştırmak
     elmayla armut kıyaslamaktır. Makale (denk. 9-10) yeniliği
         ε = y − C·x̂[k−N | k−N−1]
     yani ÖLÇÜMÜN AİT OLDUĞU ANIN tahminiyle hesaplıyor; düzeltmeyi ise
     ŞİMDİKİ duruma uyguluyor. Bu modülün tamamı bu fikrin üstüne kuruludur.

  2) ⭐ MODEL YAPISI — bilinen girdiyi kestirimden ÇIKAR (makale denk. 42).
     Makalede lineer kızağın hızı `u` girdi olarak modele giriyor, çünkü
     encoder'dan gecikmesiz biliniyor. Bizde tam karşılığı DRONE'UN KENDİ
     HIZI: göreli konumun değişiminin bir kısmı bizden, bir kısmı hedeften
     gelir; bizimkini kendi sensörümüzden tam biliyoruz. Onu girdi yapınca
     süzgecin kestirmesi gereken tek bilinmeyen HEDEFİN hızı kalır.

  3) Ölçüm denkleminin lineer tutulması (makale denk. 43, C = [1 0]).
     Kutudan (açı, açı, menzil) değil, doğrudan GÖRELİ KONUM VEKTÖRÜ
     üretiyoruz; böylece C = [I₃ 0₃] ve süzgeç gerçekten lineer kalıyor.
     Doğrusalsızlık ölçüm gürültüsü matrisine taşınır (aşağıya bak).

MAKALEDEN ALINMAYAN — ve NEDEN (dürüst ayrım)
──────────────────────────────────────────────
  ⛔ Ω/Ψ çapraz-kovaryans özyinelemesiyle "optimal kazanç" (denk. 24, 32, 40).
     Makalenin asıl matematiksel katkısı budur, ama TEK DAYANAĞI gecikmenin
     (N) ve örnekleme oranının (r) SABİT olmasıdır. Bizde ikisi de değil:
     47 uçuş / 8503 karede gecikme medyan 88 ms, p90 129 ms, maks 212 ms,
     seğirme σ = 26 ms; kare aralığı adımların %46'sında > 75 ms.
     Sabit N varsayımı yokken o türetme optimallik GARANTİ ETMEZ — yalnız
     karmaşıklık ekler.

     ⭐ YERİNE: GERİ SAR-VE-TEKRAR OYNAT (rewind & replay).
     Süzgeci ölçümün ait olduğu ana geri sarıp düzeltmeyi ORADA yapıyor,
     sonra kayıtlı girdilerle bugüne kadar yeniden ilerletiyoruz. Bu, Kalman
     süzgecinin doğru zaman sırasıyla koşturulmasından başka bir şey
     değildir — yani YAKLAŞIK DEĞİL, TAM OPTİMAL.
     Makale bunu yapmıyor çünkü onlarda N = 30 adım (10 kHz'de 30 kez
     yeniden hesap) pahalıydı; makalenin cebri o maliyetten kaçmak içindir.
     Bizde N ≈ 2 adım (20 Hz'de 95 ms) — geri sarmak bedava. Yani bizim
     halimizde daha basit olan yol aynı zamanda daha DOĞRU olan yol.

  ⛔ Ara örnekleri (inter-sample) doldurma.
     Makalenin ikinci kazanımı, kare gelmeyen adımlarda da sahte ölçüm
     üretmek. Bizim güdüm döngümüz KARE İLE SÜRÜLÜYOR (`wait_pose` yeni
     kare gelene kadar bloklar), yani "kare gelmeyen adım" diye bir tick
     yok. Bundan yararlanmak döngüyü kareden bağımsız 20 Hz'e çevirmeyi
     gerektirir — komut kadansını değiştiren AYRI ve daha riskli bir
     değişiklik. Bu adımda YAPILMADI (bkz. UYGULANACAK.md, Ö-KF adım 2).

NE ÖLÇÜYORUZ, NEREDEN GELİYOR
──────────────────────────────
  Durum (6):   x = [pn, pe, pd,  vtn, vte, vtd]
     p   : hedefin BİZE göre konumu (m), NED (kuzey-doğu-aşağı) atalet çerçevesi
     v_t : hedefin atalet hızı (m/s)
  Girdi (3):   u = drone'un kendi hızı (m/s, NED) — kendi telemetrisi
  Geçiş:
     p[k] = p[k−1] + (v_t[k−1] − u[k−1])·dt
     v_t[k] = v_t[k−1] + w          ← hedefin manevrası SÜREÇ GÜRÜLTÜSÜ sayılır
  Ölçüm (3):   y = p (kutudan türetilmiş göreli konum),  C = [I₃ 0₃]
     yön   : (cx, cy) + O KARENİN ANINDAKİ duruş → seviye azimut/yükseliş
     menzil: R = C_px_m / kutu_boyutu   (benzer üçgenler)

  ⚠ YARIŞMA KURALI §10: bu modül hedefin GPS'ine ERİŞMEZ. Girdisi yalnız
    kutu pikselleri + DRONE'UN KENDİ duruş/hız telemetrisidir.

ÖLÇÜM GÜRÜLTÜSÜ — doğrusalsızlık buraya taşındı
────────────────────────────────────────────────
  Kutudan konuma geçiş doğrusal değil; hatayı doğru büyütmek için gürültüyü
  GÖRÜŞ HATTI (LOS) çerçevesinde kurup NED'e döndürüyoruz:
     LOS boyunca (menzil yönü) :  σ_R = R²·σ_boyut / C     ← R² ile BÜYÜR
     LOS'a dik (iki yön)       :  σ_yan = R·σ_piksel / FX
  Sayıyla: C = 296.8 px·m, FX = 166.58 px, σ_boyut = 1.5 px, σ_px = 2 px
     R = 18 m →  σ_R = 1.6 m,  σ_yan = 0.22 m   (menzil 7 kat belirsiz)
     R = 40 m →  σ_R = 8.1 m,  σ_yan = 0.48 m   (menzil 17 kat belirsiz)
  Yani süzgeç uzaktayken menzile az, açıya çok güvenir — olması gereken bu.

⚠ NEYİN DOĞRU OLMADIĞI (varsayımlar ve bozulma halleri)
────────────────────────────────────────────────────────
  • Hedef SABİT HIZLI varsayılıyor; manevrası `HEDEF_IVME` süreç gürültüsüyle
    karşılanıyor. Hedef bundan sert dönerse süzgeç GERİDE KALIR (lag).
    Bu yüzden varsayılan 15 m/s² (≈1.5 g) — bilerek cömert.
  • R = C/boyut, hedefin kutusunun görünen açıklığının sabit olduğunu
    varsayar. Hedef bize göre DÖNERSE (yandan → burundan) görünen açıklık
    değişir ve menzil sıçrar. Süzgeç bunu manevra sanıp `v_t`'yi bozabilir;
    σ_R'nin R² ile büyümesi bu etkiyi kısmen bastırır.
  • τ (kare yaşı) `wall_recv`'den ölçülüyor; bu Gazebo'nun kareyi üretip
    ağdan yollamasını KAPSAMAZ. Yani gerçek gecikme ölçtüğümüzden birkaç ms
    BÜYÜK. Fark `TAU_OFSET_S` ile eklenebilir — tahminle değil, ölçümle.
  • Kutu kaybolunca süzgeç tahmine devam eder; bu "hayalet hedef" üretebilir.
    `UFUK_S` tavanı bunu keser: o süreden fazla ölçümsüz kalınırsa kestirim
    GEÇERSİZ ilan edilir ve güdüm ham yola döner.
"""

import math

try:
    import numpy as _np
    KULLANILABILIR = True
except Exception:                                    # pragma: no cover
    _np = None
    KULLANILABILIR = False


# ─────────────────────────────────────────────────────────────────────────
#  Yardımcılar — açı/piksel dönüşümleri
# ─────────────────────────────────────────────────────────────────────────

def _norm(a):
    """Açıyı (−π, π] aralığına indir."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def seviye_to_piksel(az_s, el, roll, pitch, FX, FY, CX, CY, tilt_rad):
    """`los_seviye()`'nin TAM TERSİ: seviye (azimut, yükseliş) → (cx, cy).

    `los_seviye` zinciri: piksel → kamera ışını → gövde (FRD) → seviye.
    Burada aynı zincir ters yönde koşuluyor; hiçbir küçük-açı yaklaşımı YOK.

      seviye birim vektörü:  x2 = cos(el)cos(az),  y1 = cos(el)sin(az),
                             z2 = −sin(el)
      pitch geri al:         bx = x2·cp − z2·sp ,  z1 = x2·sp + z2·cp
      roll  geri al:         by = y1·cr + z1·sr ,  bz = −y1·sr + z1·cr
      tilt  geri al:         s  = bx·ct − bz·st          (ışın ölçeği)
                             x  = by/s ,  y = (bx·st + bz·ct)/s
      piksel:                cx = CX + FX·x ,  cy = CY + FY·y

    Dönüş: (cx, cy) — hedef kameranın ARKASINDA kalıyorsa (s ≤ 0) None.
    """
    ce, se = math.cos(el), math.sin(el)
    x2 = ce * math.cos(az_s)
    y1 = ce * math.sin(az_s)
    z2 = -se
    cp, sp = math.cos(pitch), math.sin(pitch)
    bx = x2 * cp - z2 * sp
    z1 = x2 * sp + z2 * cp
    cr, sr = math.cos(roll), math.sin(roll)
    by = y1 * cr + z1 * sr
    bz = -y1 * sr + z1 * cr
    ct, st = math.cos(tilt_rad), math.sin(tilt_rad)
    s = bx * ct - bz * st
    if s <= 1e-6:
        return None                       # kameranın arkası — piksel yok
    x = by / s
    y = (bx * st + bz * ct) / s
    return CX + FX * x, CY + FY * y


def govde_elev_to_cy(el_govde, FY, CY, tilt_rad):
    """`piksel_elev()`'in tersi. Türetme:

      piksel_elev: tan(e) = (st − ct·b) / (ct + st·b),  b = (cy−CY)/FY
      çözersek:    b = (st·cos e − ct·sin e)/(ct·cos e + st·sin e)
                     = tan(tilt − e)
    """
    return CY + FY * math.tan(tilt_rad - el_govde)


# ─────────────────────────────────────────────────────────────────────────
#  Telemetri tamponu — "o karenin ANINDAKİ duruş"
# ─────────────────────────────────────────────────────────────────────────

class TelemetriTamponu:
    """Kendi duruş/hız geçmişimiz; τ kadar geri gidip o anın duruşunu verir.

    NEDEN GEREKLİ: döngü `iris` telemetrisini KOMUT anında okur, ama kare
    τ kadar eskidir. Şu anki hâlde τ eski bir piksel, taze bir duruşla
    birleştiriliyor (bu, `visual_lead.py:117-119`'da da not edilmiş bir
    bilinen hata). Doğrusu, pikseli KENDİ anının duruşuyla açıya çevirmek.
    """

    def __init__(self, kapasite=64):
        self._d = []                       # [(t, roll, pitch, yaw, vx, vy, vz)]
        self._kap = int(kapasite)

    def ekle(self, t, roll, pitch, yaw, vx, vy, vz):
        self._d.append((t, roll, pitch, yaw, vx, vy, vz))
        if len(self._d) > self._kap:
            del self._d[0]

    def bos(self):
        return not self._d

    def oku(self, t):
        """t anındaki duruş/hız — komşu iki kayıt arasında doğrusal ara değer.

        Tamponun dışına düşerse en yakın uç döner (kırpma). Yaw dairesel
        olduğu için ara değer FARK üzerinden alınır (359°↔1° sıçraması yok).
        """
        if not self._d:
            return None
        if t <= self._d[0][0]:
            return self._d[0][1:]
        if t >= self._d[-1][0]:
            return self._d[-1][1:]
        for i in range(len(self._d) - 1, 0, -1):
            t1 = self._d[i][0]
            t0 = self._d[i - 1][0]
            if t0 <= t <= t1:
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                p0, p1 = self._d[i - 1], self._d[i]
                yaw = p0[3] + a * _norm(p1[3] - p0[3])
                return (p0[1] + a * (p1[1] - p0[1]),      # roll
                        p0[2] + a * (p1[2] - p0[2]),      # pitch
                        _norm(yaw),
                        p0[4] + a * (p1[4] - p0[4]),      # vx
                        p0[5] + a * (p1[5] - p0[5]),      # vy
                        p0[6] + a * (p1[6] - p0[6]))      # vz
        return self._d[-1][1:]


# ─────────────────────────────────────────────────────────────────────────
#  Süzgeç
# ─────────────────────────────────────────────────────────────────────────

class GecikmeKF:
    """Gecikmeli ölçümlü, geri-sar-tekrar-oynat Kalman süzgeci.

    Kullanım (güdüm döngüsünde, kare başına bir kez):
        kf.ilerlet(t_simdi, u_ned)                 # tahmin — makale denk. 15
        kf.olcum(t_kare, p_rel_olculen, R_ned)     # düzeltme — denk. 9-10 + geri sar
        est = kf.kestirim(t_simdi, u_ned)          # güdüme verilecek büyüklükler
    """

    DURUM = 6

    def __init__(self, hedef_ivme=15.0, ufuk_s=0.30, tampon=64,
                 v_baslangic_sigma=25.0, red_sigma=6.0, red_tavan=4):
        if not KULLANILABILIR:
            raise RuntimeError("numpy yok — Ö-KF kullanılamaz")
        self.q = float(hedef_ivme)          # m/s²; hedefin manevra kabiliyeti
        self.ufuk_s = float(ufuk_s)
        self._kap = int(tampon)
        self._v0s = float(v_baslangic_sigma)
        self._red_sigma = float(red_sigma)
        self._red_tavan = int(red_tavan)
        self.sifirla()

    # ── yaşam döngüsü ────────────────────────────────────────────────────
    def sifirla(self):
        self.x = None                       # (6,) durum
        self.P = None                       # (6,6) kovaryans
        self.t = None                       # x'in ait olduğu an
        self._gecmis = []                   # [(t, x, P, u, )] geri sarmak için
        self.t_son_olcum = None
        self.red_ard = 0                    # arka arkaya reddedilen ölçüm
        self.son_yenilik = 0.0
        self.son_red = False
        self.olcum_sayisi = 0

    def hazir(self):
        return self.x is not None

    # ── ① TAHMİN (makale denk. 15) ───────────────────────────────────────
    def ilerlet(self, t, u):
        """Durumu t anına taşı. u = drone'un KENDİ hızı (NED, m/s)."""
        if self.x is None:
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)                   # kopukluk sonrası sıçramayı kes
        # ⚠ ARALIĞIN BAŞINDAKİ hız kullanılır (son kayıttaki u). `tekrar oynat`
        # da aynı kuralı uygular; ikisi ayrışırsa geri sarma sonucu değişirdi.
        u_ara = self._gecmis[-1][3] if self._gecmis else _np.asarray(u, float)
        self.x, self.P = self._yay(self.x, self.P, dt, u_ara)
        self.t = t
        self._gecmis.append((t, self.x.copy(), self.P.copy(),
                             _np.asarray(u, dtype=float).copy()))
        if len(self._gecmis) > self._kap:
            del self._gecmis[0]

    def _yay(self, x, P, dt, u):
        """x[k] = A·x[k−1] + B·u  ;  P[k] = A·P·Aᵀ + Q  (makale denk. 15, 17)"""
        u = _np.asarray(u, dtype=float)
        A = _np.eye(6)
        A[0:3, 3:6] = _np.eye(3) * dt
        xn = A @ x
        xn[0:3] -= u * dt                   # B·u — bizim kendi hareketimiz
        # Q: sürekli beyaz-gürültülü ivme modelinin ayrık karşılığı
        q2 = self.q * self.q
        Q = _np.zeros((6, 6))
        Q[0:3, 0:3] = _np.eye(3) * (q2 * dt ** 4 / 4.0)
        Q[0:3, 3:6] = _np.eye(3) * (q2 * dt ** 3 / 2.0)
        Q[3:6, 0:3] = _np.eye(3) * (q2 * dt ** 3 / 2.0)
        Q[3:6, 3:6] = _np.eye(3) * (q2 * dt ** 2)
        return xn, A @ P @ A.T + Q

    # ── ② DÜZELTME — gecikmeli ölçüm (makalenin ana fikri) ───────────────
    def olcum(self, t_olcum, y, R_olcum, u_simdi):
        """τ kadar ESKİ bir konum ölçümünü doğru çağa oturtarak işle.

        Adımlar:
          1. Süzgeci ölçümün ait olduğu ana GERİ SAR (geçmiş tamponundan).
          2. Standart Kalman düzeltmesini ORADA yap  →  ε doğru çağla eşlendi.
          3. Kayıtlı girdilerle bugüne kadar yeniden ilerlet.
        Bu, makalenin denk. 9-10'daki "yeniliği geçmiş tahminle hesapla,
        düzeltmeyi şimdiye uygula" fikrinin TAM (yaklaşıksız) karşılığıdır.

        Dönüş: True = ölçüm işlendi, False = reddedildi/kullanılamadı.
        """
        y = _np.asarray(y, dtype=float)
        self.son_red = False
        # ── ilk ölçüm: süzgeci başlat ────────────────────────────────────
        if self.x is None:
            self.x = _np.zeros(6)
            self.x[0:3] = y
            self.P = _np.zeros((6, 6))
            self.P[0:3, 0:3] = R_olcum
            # hedefin hızı bilinmiyor → ÇOK geniş başla, ilk karelerde otursun
            self.P[3:6, 3:6] = _np.eye(3) * (self._v0s ** 2)
            self.t = t_olcum
            self._gecmis = [(t_olcum, self.x.copy(), self.P.copy(),
                             _np.asarray(u_simdi, dtype=float).copy())]
            self.t_son_olcum = t_olcum
            self.olcum_sayisi = 1
            return True

        simdi = self.t
        if t_olcum > simdi:                 # kare "gelecekten" — çağı kırp
            t_olcum = simdi
        # 1) geri sar: t_olcum'dan ÖNCEKİ son anlık görüntüyü bul
        j = None
        for i in range(len(self._gecmis) - 1, -1, -1):
            if self._gecmis[i][0] <= t_olcum:
                j = i
                break
        if j is None:
            # ölçüm tamponun tamamından eski — BAYAT, at
            self.son_red = True
            return False
        t_j, x_j, P_j, u_j = self._gecmis[j]
        x, P = x_j.copy(), P_j.copy()
        if t_olcum > t_j:
            x, P = self._yay(x, P, t_olcum - t_j, u_j)

        # 2) düzeltme — C = [I₃ 0₃] olduğu için alt-blok cebri yeter
        H = _np.zeros((3, 6))
        H[0:3, 0:3] = _np.eye(3)
        eps = y - x[0:3]                    # ⭐ YENİLİK, doğru çağla
        S = P[0:3, 0:3] + R_olcum
        try:
            Sinv = _np.linalg.inv(S)
        except Exception:
            self.son_red = True
            return False
        # aykırı ölçüm kapısı (Mahalanobis) — cömert; ısrarla reddedilirse
        # süzgeç sıfırlanır, yoksa kalıcı olarak kilitlenebilirdi
        d2 = float(eps @ Sinv @ eps)
        self.son_yenilik = float(_np.linalg.norm(eps))
        if d2 > self._red_sigma ** 2 * 3.0:
            self.red_ard += 1
            self.son_red = True
            if self.red_ard >= self._red_tavan:
                self.sifirla()              # kilitlenme koruması
                return self.olcum(t_olcum, y, R_olcum, u_simdi)
            return False
        self.red_ard = 0
        K = P @ H.T @ Sinv                  # (6,3) Kalman kazancı
        x = x + K @ eps
        I_KH = _np.eye(6) - K @ H
        P = I_KH @ P @ I_KH.T + K @ R_olcum @ K.T   # Joseph — simetri korunur

        # 3) tekrar oynat: kayıtlı girdilerle bugüne ilerlet
        t_cur = t_olcum
        yeni_gecmis = [(t_cur, x.copy(), P.copy(), u_j.copy())]
        for i in range(j + 1, len(self._gecmis)):
            t_i, _, _, u_i = self._gecmis[i]
            if t_i > t_cur:
                x, P = self._yay(x, P, t_i - t_cur, yeni_gecmis[-1][3])
                t_cur = t_i
            yeni_gecmis.append((t_i, x.copy(), P.copy(), u_i.copy()))
        if simdi > t_cur:
            x, P = self._yay(x, P, simdi - t_cur, yeni_gecmis[-1][3])
            t_cur = simdi
            yeni_gecmis.append((t_cur, x.copy(), P.copy(),
                                _np.asarray(u_simdi, dtype=float).copy()))
        self.x, self.P, self.t = x, P, t_cur
        self._gecmis = yeni_gecmis[-self._kap:]
        self.t_son_olcum = t_olcum
        self.olcum_sayisi += 1
        return True

    # ── ③ ÇIKTI — güdümün tükettiği büyüklükler ─────────────────────────
    def kestirim(self, t, u):
        """t anındaki kestirimden güdüm büyüklüklerini üret.

        Dönüş sözlüğü (hepsi ATALET çerçevesinde):
          gecerli   : kestirime güvenilir mi (ufuk tavanı + ısınma)
          az, el    : görüş hattı açıları (rad) — az kuzeyden saat yönü +
          R         : menzil (m)
          lam_az/el : LOS dönüş hızları λ̇ (rad/s)
          kapanma   : −dR/dt (m/s, + = yaklaşıyor)
          ileri_s   : son ölçümden bu yana geçen süre (tahminle gidilen)
          P_iz      : Tr(P) — süzgecin kendi belirsizliği
        """
        bos = {"gecerli": False, "az": 0.0, "el": 0.0, "R": 0.0,
               "lam_az": 0.0, "lam_el": 0.0, "kapanma": 0.0,
               "ileri_s": 0.0, "P_iz": 0.0, "yenilik": self.son_yenilik}
        if self.x is None:
            return bos
        x, P = self.x, self.P
        if t > self.t:                      # komut anına kadar taşı
            x, P = self._yay(x, P, min(t - self.t, 0.5), u)
        p = x[0:3]
        v_rel = x[3:6] - _np.asarray(u, dtype=float)   # hedef hızı − bizim hız
        R = float(_np.linalg.norm(p))
        ileri = (t - self.t_son_olcum) if self.t_son_olcum is not None else 1e9
        if R < 0.05 or not _np.isfinite(R):
            return bos
        yatay2 = float(p[0] ** 2 + p[1] ** 2)
        if yatay2 < 1e-6:
            return bos
        az = math.atan2(float(p[1]), float(p[0]))
        el = math.atan2(float(-p[2]), math.sqrt(yatay2))
        # λ̇_az = d/dt atan2(pe, pn) = (pn·ve − pe·vn)/(pn² + pe²)
        lam_az = (float(p[0]) * float(v_rel[1])
                  - float(p[1]) * float(v_rel[0])) / yatay2
        # λ̇_el = d/dt atan2(−pd, √(pn²+pe²))
        yatay = math.sqrt(yatay2)
        ydot = (float(p[0]) * float(v_rel[0])
                + float(p[1]) * float(v_rel[1])) / yatay
        lam_el = (-float(v_rel[2]) * yatay + float(p[2]) * ydot) / (R * R)
        kapanma = -float(p @ v_rel) / R
        return {
            "gecerli": (self.olcum_sayisi >= 3 and ileri <= self.ufuk_s),
            "az": az, "el": el, "R": R,
            "lam_az": lam_az, "lam_el": lam_el, "kapanma": kapanma,
            "ileri_s": float(ileri), "P_iz": float(_np.trace(P)),
            "yenilik": self.son_yenilik,
        }


# ─────────────────────────────────────────────────────────────────────────
#  Ölçüm gürültüsü — doğrusalsızlığı R matrisine taşıyan yer
# ─────────────────────────────────────────────────────────────────────────

def olcum_kovaryansi(p_birim, R_menzil, sigma_menzil, sigma_yanal):
    """LOS çerçevesinde köşegen gürültüyü NED'e döndür.

    R_ned = M·diag(σ_R², σ_yan², σ_yan²)·Mᵀ
    M'nin ilk sütunu LOS birim vektörü, diğer ikisi ona dik iki eksen.
    Böylece menzil belirsizliği (büyük) LOS BOYUNCA, açı belirsizliği
    (küçük) LOS'A DİK kalır — elipsoid gerçek hata dağılımına oturur.
    """
    u1 = _np.asarray(p_birim, dtype=float)
    n = _np.linalg.norm(u1)
    if n < 1e-9:
        return _np.eye(3) * max(sigma_menzil, sigma_yanal) ** 2
    u1 = u1 / n
    yardim = _np.array([0.0, 0.0, 1.0])
    if abs(float(u1 @ yardim)) > 0.95:
        yardim = _np.array([1.0, 0.0, 0.0])
    u2 = _np.cross(u1, yardim)
    u2 /= _np.linalg.norm(u2)
    u3 = _np.cross(u1, u2)
    M = _np.column_stack([u1, u2, u3])
    D = _np.diag([sigma_menzil ** 2, sigma_yanal ** 2, sigma_yanal ** 2])
    return M @ D @ M.T
