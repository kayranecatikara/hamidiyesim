# DAİRE TEŞHİSİ — tek uçuşun kare kare analizi (2026-08-15)

Beş kampanya (Ö5, Ö11, Ö-B×2, Ö-J) sayı toplayıp hipotez üretti ve beşi de
tutmadı. Bu kez CLAUDE.md §2.4'ün dediği yapıldı: **tek uçuş, kareleri gözle.**
Koşu: `logs/kayit/daire_analiz` — circle senaryosu, 200 s, 201 kare + 20 Hz log.

## 1 · DESEN: mesafe salınıyor, hedefin ÖNÜNE düşüyoruz

mesafe 18-40 m (yakın) ↔ 85-120 m (uzak), periyot ~30-40 s.
kuyruk açısı 17-55° (arkada) ↔ **130-170° (hedefin ÖNÜNDE)**.

Yani yaklaşıyoruz, hedefi geçiyoruz, önüne düşüyoruz, mesafe 100 m'ye
açılıyor, tekrar yaklaşıyoruz. Klasik saf takip aşımı — dönen hedefte
köşeyi kesip önden çıkıyoruz.

## 2 · EN SERT SAYI: kuyruk konisine HİÇ giremiyoruz

200 saniyelik uçuşta, **60 m içinde VE kuyruk açısı 0-30°** olan kare:
**SIFIR.**
(Kıyas: `duz` senaryosunda karelerin %92'si 0-30° kuyrukta.)

Dağılım (60 m içindeki kareler): 30-60° 22 · 60-90° 18 · 90-120° 10 ·
120-180° **27**. Yani en kalabalık dilim hedefin ÖNÜ.

## 3 · SONUCU: faz savrulması

| senaryo | faz geçişi / 200 s | görsel fazda kutu var % | kutusuz süre |
|---|---|---|---|
| circle | **19-30** | 34-39% | 45-67 s |
| square | 19 | 33% | 72 s |
| **duz** | **4-8** | **59-75%** | 6-20 s |

Manevrada 20-30 kez GPS↔görsel gidip geliyoruz. Görsel faza devrediyoruz,
kutuyu saniyeler içinde kaybediyoruz (`KUTU_YOK` 67 s), kör hücuma
düşüyoruz (`TERM_KOR` 21 s), vazgeçip GPS'e dönüyoruz, yeniden yakalıyoruz.

Gözle bakılan kareler bunu doğruluyor: kare 121 (mesafe **17.8 m**) ve
kare 124'te hedef kadrajda **YOK**.

## 4 · BUNUN ANLAMI

Beş kampanyadır ayarladığım her şey (hız, mandal, yatış, araç parametreleri)
**GÖRSEL FAZIN İÇİNDE** çalışıyor. Ama dairede görsel faz zamanın 2/3'ünde
KÖR. Yani doğru katmanı hiç ayarlamamışım.

Dairede kaybetme sebebi: **kuyruk konisine hiç yerleşemiyoruz** → görsel
temas kurulamıyor → faz savruluyor.

## 5 · SIRADAKİ SORU (henüz öneri DEĞİL)

Neden kuyruğa yerleşemiyoruz? İki aday, ikisi de ölçülmeli:
  (a) İstasyon hedefin hız yönünün 8 m gerisine kuruluyor. Dönen hedefte
      bu nokta yay üzerinde kaçıyor; saf takiple kovalayınca köşe kesip
      önden çıkıyoruz. → istasyonun YAYA göre konması gerekebilir.
  (b) Kamera gövdeye sabit; yanal aspect'te (76° medyan) hedefin silueti
      kuyruk görünümünden farklı — dedektör kaçırıyor olabilir.
      Ölçülmedi: kutu varlığı ↔ aspect açısı ilişkisi izole edilmedi.
