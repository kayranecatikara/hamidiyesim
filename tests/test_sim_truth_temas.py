# tests/test_sim_truth_temas.py — sim_truth temas (contact) mantığı birim testleri.
# gz gerekmez: _temas_cb sahte Contacts mesajlarıyla doğrudan sürülür.
import types

from control import sim_truth


def _msg(*cift_listesi):
    """cift_listesi: (collision1_adi, collision2_adi) çiftleri → sahte Contacts."""
    m = types.SimpleNamespace(contact=[])
    for a, b in cift_listesi:
        m.contact.append(types.SimpleNamespace(
            collision1=types.SimpleNamespace(name=a),
            collision2=types.SimpleNamespace(name=b)))
    return m


def _temiz():
    sim_truth.temas_sifirla()
    with sim_truth._lock:
        sim_truth._temas["mevcut"] = True   # akış kurulu varsay


def test_yer_temasi_vurus_sayilmaz():
    _temiz()
    # Park hâlinde/inişte: talon yalnız yerle temas ediyor
    sim_truth._temas_cb(_msg(("mini_talon::base_link::fuselage_collision",
                              "ground_plane::link::collision")))
    assert sim_truth.temas() is False


def test_iris_temasi_vurus_ve_latch():
    _temiz()
    sim_truth._temas_cb(_msg(
        ("mini_talon::base_link::left_wing_collision",
         "iris_with_ardupilot::base_link::base_link_collision")))
    assert sim_truth.temas() is True
    # Latch: temas mesajı kesilse de True kalır
    sim_truth._temas_cb(_msg())
    assert sim_truth.temas() is True
    # Reset sonrası temiz
    sim_truth.temas_sifirla()
    assert sim_truth.temas() is False


def test_iris_collision1_tarafinda_da_yakalanir():
    _temiz()
    sim_truth._temas_cb(_msg(
        ("iris_with_ardupilot::base_link::rotor_collision",
         "mini_talon::base_link::fuselage_collision")))
    assert sim_truth.temas() is True


def test_akis_yoksa_none():
    sim_truth.temas_sifirla()
    with sim_truth._lock:
        sim_truth._temas["mevcut"] = False
    assert sim_truth.temas() is None
