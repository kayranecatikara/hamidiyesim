#!/usr/bin/env bash
# Mermi govdeli ag atan taretli interceptor'i GUI'de acar ve kamerayi
# drone'u gorecek sekilde konumlandirir.
#   ./GOSTER.sh
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/env.sh"
W=bullet_net_test

# Zaten acik bir kosum varsa kapat (ag tek atimlik, temiz baslamak gerek)
# Acik kosumlari kapat. "gz sim" aslinda bir ruby sureci; comm=gz ile
# eslesmiyor, bu yuzden tam komut satirina bakiyoruz.
pkill -f "^gz sim" 2>/dev/null
for _ in $(seq 10); do pgrep -f "^gz sim" >/dev/null || break; sleep 0.5; done

echo ">> Gazebo aciliyor..."
gz sim -r "$ROOT/worlds/$W.sdf" >/tmp/gz_goster.log 2>&1 &

for _ in $(seq 60); do
  gz topic -l 2>/dev/null | grep -q "/world/$W/stats" && break
  sleep 0.5
done
if ! gz topic -l 2>/dev/null | grep -q "/world/$W/stats"; then
  echo "HATA: dunya acilmadi. Log: /tmp/gz_goster.log"; tail -20 /tmp/gz_goster.log; exit 1
fi

# GUI'nin sahneyi kurmasini bekle, sonra kamerayi yerlestir.
# Ilk istek sahne hazir olmadan gidebiliyor; bu yuzden birkac kez gonderiyoruz.
# GUI acilinca dunya DURAKLATILMIS geliyor; acikca calistir.
gz service -s "/world/$W/control" --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >/dev/null 2>&1

# Pencereyi buyut (kucuk acilirsa sahne dar gorunuyor)
if command -v wmctrl >/dev/null 2>&1; then
  sleep 2
  WID=$(wmctrl -l | grep -i "Gazebo Sim" | head -1 | cut -d' ' -f1)
  [ -n "$WID" ] && wmctrl -i -r "$WID" -b add,maximized_vert,maximized_horz 2>/dev/null
fi

KAMERA='pose: {position: {x: -4.0, y: -3.0, z: 2.0}, orientation: {x: -0.0208, y: 0.1015, z: 0.2019, w: 0.9740}}'
for _ in 1 2 3; do
  sleep 3
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean --timeout 3000 --req "$KAMERA" >/dev/null 2>&1
done

cat <<'MSG'

=== HAZIR ===
Pencerede: dikey mermi govde, tepesinde taret, namluda ag konisi,
3 m ileride kirmizi hedef kutusu.

Simdi BU terminalde su komutlari sirayla calistir:

  source scripts/env.sh
  python3 scripts/turret_aim.py 0 -8 --model bullet_net_interceptor
  python3 scripts/fire_net.py --hiz 20 --model bullet_net_interceptor

Taret limitleri: pan +-100 derece, tilt -60..+30 (negatif = yukari).
Ag TEK ATIMLIK -- tekrar atmak icin ./GOSTER.sh yeniden calistir.

NOT: Sol alttaki oynat/duraklat dugmesi DURAKLATILMIS olmamali.
Duraklatilmisken atis komutu birikir ve devam edince ag ters yone firlar.
MSG
