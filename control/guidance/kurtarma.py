"""
kurtarma.py — UÇUŞ KURTARMA BEKÇİSİ (güdümden bağımsız emniyet katmanı).

⚠ NE YAPAR: aracın kontrolü kaybettiğini (takla / kaçak dönme) tespit eder ve
o süre boyunca güdüm komutlarını KESİP aracı toparlanmaya bırakır. Toparlanınca
kontrolü güdüme geri verir.

⚠ NE YAPMAZ: normal uçuşta HİÇBİR ŞEYE dokunmaz. Güdüm yasalarını, hız/yaw
limitlerini, istasyon geometrisini değiştirmez. Eşikler ölçülmüş uçuş zarfının
ÇOK dışında seçilmiştir (aşağıya bak) — sağlıklı uçuşta asla tetiklenmez.

NEDEN VAR (2026-08-08, kullanıcı bildirdi + logdan doğrulandı):
Hedefi ıskalayıp geçtikten sonra araç kontrolden çıkıp kendi etrafında dönerek
düşüyordu. Log 200116 kanıtı — güdüm SAKİN komut verirken (yaw komutu düzgün
slew ediyor, hız 18 m/s sabit) aracın GERÇEK duruşu:
    roll  −97° → −111° → −119° → −157° (TERS DÖNDÜ)
    yaw hızı 3590 °/s (saniyede 10 tur)
    irtifa 97.8 m → 5.3 m → yer altı (12 saniyede)
Yani sorun güdümün ne istediği değil, aracın isteneni uygulayamaz hale gelmesi.
Bir kez bu duruma girince güdüm komut vermeye devam ettiği için araç
toparlanamıyor ve uçuş bitiyor — aynı koşuda ikinci deneme şansı kalmıyor.

EŞİKLER — ölçülmüş sağlıklı zarf (3 temiz uçuş: 172225, 134512, 121248):
    roll   en fazla 46°   |  pitch en fazla 33°  |  yaw hızı en fazla 188°/s
Eşikler bunların belirgin üstünde: 60° ve 300°/s. Test K3 bu zarfın
tetiklemediğini kalıcı olarak bekçiliyor.
"""

import math
import os


def _env_f(name, default):
    return float(os.environ.get(name, default))


class KurtCfg:
    # Tetik eşikleri — ölçülen sağlıklı zarfın (46°, 188°/s) BELİRGİN üstünde.
    ACI_TETIK = _env_f("AVCI_KURT_ACI", 60.0)        # °; |roll| veya |pitch|
    YAW_HIZ_TETIK = _env_f("AVCI_KURT_YAW", 300.0)   # °/s

    # Çıkış (toparlandı) ölçütleri — tetikten çok daha sıkı (histerezis).
    ACI_TEMIZ = 20.0          # °
    YAW_HIZ_TEMIZ = 60.0      # °/s
    TEMIZ_SURE = 0.7          # s; bu kadar süre temiz kalmalı

    # UYARI eşiği — bekçi BIRAKMAZ, yalnız log basar.
    # ⚠ İlk tasarımda "bu süre dolunca güdüme bırak" vardı; test K6 bunun
    # LIVELOCK olduğunu gösterdi: duruş hâlâ kötü olduğu için bir sonraki
    # turda anında yeniden tetikleniyordu. Zaten yanlış fikirdi — takla atan
    # araca komut vermek onu öldüren şeydi. Duruş düzelene kadar komut kesik
    # kalır; düzelmiyorsa araç kurtarılamıyor demektir ve komut vermek bunu
    # değiştirmez.
    UYARI_SURE = 8.0          # s

    AKTIF = _env_f("AVCI_KURT", 1.0) >= 0.5   # 0 = bekçi kapalı (eski davranış)


class Kurtarma:
    """Duruş bekçisi. Her güdüm turunda guncelle() çağrılır.

    Kullanım:
        kurt = Kurtarma()
        ...
        if kurt.guncelle(roll, pitch, yaw, now):
            send_velocity(conn, 0.0, 0.0, 0.0, yaw)   # hız kes, yaw'ı yerinde tut
            continue                                   # güdüm bu turu atlar
    """

    def __init__(self, cfg=KurtCfg):
        self.cfg = cfg
        self.aktif = False
        self._yaw_onceki = None
        self._t_onceki = None
        self._temiz_baslangic = None
        self._baslangic = None
        self._uyardi = False
        self.sayac = 0            # kaç kez devreye girdi (log/teşhis)
        self.son_sebep = None

    def yaw_hizi(self, yaw, now):
        """Ardışık yaw'dan açısal hız (°/s). İlk turda 0."""
        if self._yaw_onceki is None or self._t_onceki is None:
            return 0.0
        dt = now - self._t_onceki
        if dt <= 1e-4 or dt > 1.0:
            return 0.0
        d = math.degrees(
            (yaw - self._yaw_onceki + math.pi) % (2 * math.pi) - math.pi)
        return d / dt

    def guncelle(self, roll, pitch, yaw, now):
        """roll/pitch/yaw RADYAN, now = monotonic saniye.
        Dönüş: True = kurtarma aktif, güdüm komutları KESİLMELİ."""
        c = self.cfg
        if not c.AKTIF:
            return False

        hiz = self.yaw_hizi(yaw, now)
        self._yaw_onceki, self._t_onceki = yaw, now

        aci = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
        kotu = (aci > c.ACI_TETIK) or (abs(hiz) > c.YAW_HIZ_TETIK)

        if not self.aktif:
            if kotu:
                self.aktif = True
                self.sayac += 1
                self._baslangic = now
                self._temiz_baslangic = None
                self._uyardi = False
                self.son_sebep = (f"açı {aci:.0f}°" if aci > c.ACI_TETIK
                                  else f"yaw {abs(hiz):.0f}°/s")
                print(f"[KURTARMA] ⚠ kontrol kaybı ({self.son_sebep}) — güdüm "
                      f"komutları kesildi, araç toparlanıyor")
            return self.aktif

        # aktifken: çıkış koşulu
        temiz = (aci < c.ACI_TEMIZ) and (abs(hiz) < c.YAW_HIZ_TEMIZ)
        if temiz:
            if self._temiz_baslangic is None:
                self._temiz_baslangic = now
            elif now - self._temiz_baslangic >= c.TEMIZ_SURE:
                sure = now - self._baslangic
                self.aktif = False
                print(f"[KURTARMA] ✓ toparlandı ({sure:.1f} s) — güdüm devraldı")
        else:
            self._temiz_baslangic = None

        if (self.aktif and not self._uyardi
                and (now - self._baslangic) > c.UYARI_SURE):
            self._uyardi = True
            print(f"[KURTARMA] ⚠ {c.UYARI_SURE:.0f} s'dir toparlanamıyor — "
                  f"komut kesik kalmaya devam ediyor (araç kurtarılamıyor "
                  f"olabilir)")
        return self.aktif
