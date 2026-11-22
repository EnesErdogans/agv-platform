# AGV – Tek komutla başlat

Kamera + LiDAR haritası + Windows uygulaması köprüsü. Hepsi tek komutla başlar, tek sayfada görünür.

## 1. Klasörü Pi'ye kopyala (BİLGİSAYARDA)

Windows'ta **PowerShell** aç, `agv_pi` klasörünün olduğu yere (ör. İndirilenler) git ve kopyala:

```powershell
cd $HOME\Downloads
scp -r agv_pi agv@<PI-IP>:~/
```

> `scp` komutu **bilgisayarda** çalışır, SSH içinde (Pi'de) değil. Örnek IP: `192.168.1.50`.

## 2. Başlat (Pi'de)

```bash
ssh agv@<PI-IP>
bash ~/agv_pi/agv.sh
```

- İlk çalıştırmada Docker imajı derlenir. Birkaç dakika sürer. Sonraki açılışlar hızlıdır.
- Sonunda ekranda adresler yazar. Tarayıcıda aç: **http://<PI-IP>:8080**
  (sol tarafta canlı harita, sağ tarafta kamera ve tespit edilen nesneler)
- Windows uygulaması: adres `<PI-IP>`, port `8765`.
- Komutu iki kere çalıştırmak sorun değil. Her şeyi temizleyip baştan başlatır.

## Komutlar

| Komut | Ne yapar |
|---|---|
| `bash ~/agv_pi/agv.sh` | Her şeyi başlat |
| `bash ~/agv_pi/agv.sh dur` | Her şeyi durdur |
| `bash ~/agv_pi/agv.sh durum` | Ne çalışıyor? LiDAR kaç Hz? Portlar açık mı? |
| `bash ~/agv_pi/agv.sh log` | Kamera ve konteyner loglarının son satırları |
| `bash ~/agv_pi/agv.sh otomatik` | Pi açılınca her şey kendiliğinden başlasın |
| `bash ~/agv_pi/agv.sh otomatik-kapat` | Otomatik başlatmayı kaldır |

## Paneldeki butonlar

- **Haritayı kaydet**: bir ad sorar. Harita `~/maps/<ad>.pgm` ve `.yaml` olarak kaydedilir.
- **Haritayı sil**: mevcut haritayı siler, SLAM sıfırdan başlar (birkaç saniye sürer). Önce kaydetmeyi unutma.
- **Kayıtlı haritalar**: `~/maps` içindeki haritaların listesi.

## Sorun giderme

- **LiDAR hatası `80008002`** veya panelde "LiDAR: veri yok": LiDAR'ın USB kablosunu çıkar, 5 sn bekle, tekrar tak.
  Sonra `bash ~/agv_pi/agv.sh durum` ile kontrol et.
- **Kamera çevrimdışı**: `tail -n 30 ~/agv_pi/logs/kamera.log` ile hatayı gör. Kamera kablosunu kontrol et.
  Kamera çalışmasa bile LiDAR ve harita çalışmaya devam eder.
- Bir şey ters giderse: `bash ~/agv_pi/agv.sh log`

## Eski dosyalar

Eski `~/lidar_slam` klasörüne ve elle başlatılan eski konteynere artık gerek yok. `agv.sh` eski
`agv-lidar` konteynerini kendisi kaldırıp yenisini başlatır. Eski kamera programını (`kamera_ai.py`) da kendisi
kapatır. İstersen eski klasörü silebilirsin: `rm -rf ~/lidar_slam` (haritalar `~/maps` içinde, silinmez).
