"""tests/olcum_araclari — ÖLÇÜM ARAÇLARININ kendi testleri.

Buradaki testler güdümü değil, `tools/` altındaki ölçüm araçlarını denetler.

NEDEN AYRI KLASÖR: bu araçlar "vurduk mu, nereden ıskaladık" sorusunun
HAKEMİ. Güdüm testleri (tests/test_visual_lead.py, test_gps_guidance.py)
kodun doğru davrandığını gösterir; buradakiler ÖLÇÜMÜN doğru olduğunu
gösterir. Hakem sessizce yanılırsa ona dayanan her karar bozulur ve kimse
fark etmez — nitekim CSV'ye güvenilip 3.20 m sanılan bir vuruş kara kutuda
0.21 m çıkmıştı.

KAPSAM SINIRI: araçların kendisi uçuş kaydı okur (`~/ardupilot/logs/*.BIN`,
`logs/*.csv`) ve bunlar depoda YOK. O yüzden burada I/O değil, araçların
İÇİNDEKİ SAF HESAP sentetik veriyle test edilir — zaman hizalama, geometri,
metrik toplama, parametre karşılaştırma. "Test koştur, ölçüm gelsin" diye
bir şey yoktur; ölçüm için uçmak gerekir.

    python3 -m tests.olcum_araclari              # hepsi
    python3 -m tests.olcum_araclari.test_gecis_analiz   # tek dosya
"""
