# STM32 CNC Titreşim İzleme Sistemi

STM32F407 Discovery kartı üzerinde geliştirilen, CMSIS-DSP kütüphanesi kullanılarak gerçek zamanlı titreşim analizi ve rulman arıza frekansı izleme işlemlerini gerçekleştiren gömülü yazılım projesidir.

Bu proje; CNC tezgâhları, spindle sistemleri ve döner makinelerde oluşabilecek rulman kaynaklı arızaların erken tespitine yönelik bir durum izleme (Condition Monitoring) altyapısı sunmaktadır.

---

## Proje Hakkında

Döner makinelerde meydana gelen rulman arızaları, üretim kayıplarına, plansız duruşlara ve yüksek bakım maliyetlerine neden olmaktadır. Titreşim analizi, bu arızaların erken aşamada tespit edilmesinde yaygın olarak kullanılan yöntemlerden biridir.

Bu projede STM32F407 mikrodenetleyicisi kullanılarak titreşim verileri gerçek zamanlı olarak toplanmakta, FFT analizi uygulanmakta ve elde edilen sonuçlar UART üzerinden masaüstü uygulamasına aktarılmaktadır.

Sistem, gömülü tarafta veri toplama ve sinyal işleme görevlerini üstlenirken, görselleştirme ve kullanıcı arayüzü işlemleri ayrı bir Qt uygulaması tarafından gerçekleştirilmektedir.

---

## Donanım Bileşenleri

### Geliştirme Kartı

* STM32F407 Discovery

### Sensör

* LIS3DSH Üç Eksenli İvmeölçer

### Haberleşme

* UART / USART

---

## Kullanılan Teknolojiler

* STM32CubeIDE
* STM32 HAL Kütüphaneleri
* CMSIS-DSP
* C Programlama Dili

---

## Özellikler

### Titreşim Verisi Toplama

* X, Y ve Z eksenlerinden ivme verisi okuma
* SPI üzerinden sensör haberleşmesi
* Gerçek zamanlı veri toplama

### Sinyal İşleme

* Bileşke titreşim hesabı
* RMS hesaplama
* Baskın frekans tespiti
* Tepe genlik (Peak Amplitude) analizi

### FFT Analizi

* CMSIS-DSP tabanlı FFT uygulaması
* Gerçek zamanlı frekans spektrumu oluşturma
* Gömülü sistem üzerinde frekans domeni analizi

### Rulman Arıza Analizi

Aşağıdaki karakteristik rulman arıza frekanslarının hesaplanması ve izlenmesi desteklenmektedir:

* BPFO (Dış Bilezik Arızası)
* BPFI (İç Bilezik Arızası)
* BSF (Bilye Arızası)
* FTF (Kafes Arızası)

### UART Veri Aktarımı

STM32 tarafında üretilen analiz sonuçları UART üzerinden bilgisayara aktarılır.

Desteklenen veri paketleri:

```text
RAW
FFT
SPEC
FAULT_FREQ
```

---

## Sistem Mimarisi

```text
LIS3DSH İvmeölçer
        │
        ▼
   STM32F407
        │
        ▼
 CMSIS-DSP FFT
        │
        ▼
 UART Haberleşmesi
        │
        ▼
 Qt Masaüstü Arayüzü
        │
        ▼
 Arıza Analizi ve İzleme
```

---

## Proje Yapısı

```text
Core/
Drivers/
Middlewares/
USB_HOST/
Debug/

STM32F407VGTX_FLASH.ld
STM32F407VGTX_RAM.ld
yeni_usart.ioc
```

---

## FFT Yapılandırması

```text
FFT Boyutu        : 512
Örnekleme Hızı    : 1600 Hz
Maksimum Frekans  : 800 Hz
```

Nyquist kriteri nedeniyle analiz edilebilen maksimum frekans yaklaşık 800 Hz ile sınırlıdır.

---

## Derleme ve Yükleme

1. Projeyi klonlayın.

```bash
git clone https://github.com/<melih-fndk>/stm32-cnc-vibration-monitor.git
```

2. STM32CubeIDE ile projeyi açın.

3. Gerekirse `.ioc` dosyasından kod üretimini gerçekleştirin.

4. Projeyi derleyin.

5. Firmware'i STM32F407 karta yükleyin.

6. UART bağlantısını kurarak masaüstü uygulaması ile haberleşmeyi başlatın.

---

## Masaüstü Uygulaması

Bu depo yalnızca STM32 gömülü yazılımını içermektedir.

Veri görselleştirme ve kullanıcı arayüzü işlemleri ayrı bir Qt uygulaması tarafından gerçekleştirilmektedir.

Proje yapısı:

```text
stm32-cnc-vibration-monitor  → STM32 Firmware
stm32-uart-reader            → Qt Masaüstü Uygulaması
```

---

## Mevcut Durum

* UART haberleşmesi tamamlandı
* LIS3DSH sensör entegrasyonu tamamlandı
* Gerçek zamanlı titreşim verisi toplama tamamlandı
* CMSIS-DSP entegrasyonu tamamlandı
* FFT altyapısı tamamlandı
* Qt haberleşme protokolü oluşturuldu

---

## Gelecek Çalışmalar

* DMA tabanlı veri toplama
* Daha yüksek örnekleme frekansı desteği
* Envelope Analysis
* Kurtosis ve Crest Factor hesaplamaları
* Veri kayıt sistemi
* Fault Score algoritmaları
* Makine öğrenmesi tabanlı sınıflandırma
* Kestirimci bakım (Predictive Maintenance) uygulamaları

---

## Lisans

Bu proje eğitim, araştırma ve durum izleme uygulamaları amacıyla geliştirilmektedir.

---

**Geliştirici:** Melih Fındık

**Platform:** STM32F407 Discovery

**Uygulama Alanı:** CNC, spindle ve döner makinelerde titreşim analizi ve rulman arıza izleme sistemleri.

## Geliştirme Durumu

Bu depo şu anda yalnızca STM32 tabanlı gömülü yazılım kısmını içermektedir.

Qt tabanlı masaüstü izleme uygulaması ilerleyen sürümlerde ayrı bir depo olarak yayımlanacaktır.




STM32 CNC Vibration Monitor

Real-time vibration monitoring and bearing fault analysis firmware developed on STM32F407 Discovery using CMSIS-DSP and LIS3DSH accelerometer.

This project is the embedded side of a condition monitoring system designed for CNC machines, electric motors, and spindle applications. The firmware acquires vibration data, performs real-time signal processing and FFT analysis, calculates bearing characteristic frequencies, and transmits the results to a desktop application via UART.

Overview

Unexpected bearing failures are among the most common causes of downtime in rotating machinery. Early detection of vibration anomalies can significantly reduce maintenance costs and improve system reliability.

This firmware implements a lightweight embedded vibration analysis pipeline on STM32F407, providing:

Real-time vibration acquisition
FFT-based frequency analysis
RMS and peak detection
Bearing fault frequency calculations
UART communication with external monitoring software

The processed data can be visualized and analyzed using the companion Qt desktop application.

Hardware
Development Board
STM32F407 Discovery
Sensor
LIS3DSH 3-Axis Accelerometer
Communication
USART/UART over USB
Software Stack
STM32CubeIDE
STM32 HAL Drivers
CMSIS-DSP Library
C Language
Features
Sensor Acquisition
Real-time X, Y and Z axis acceleration measurement
SPI communication with LIS3DSH
Continuous vibration sampling
Signal Processing
Resultant vibration magnitude calculation
RMS computation
Peak amplitude detection
Peak frequency detection
FFT Analysis
CMSIS-DSP based FFT implementation
Real-time frequency spectrum generation
Embedded frequency-domain processing
Bearing Fault Monitoring

Characteristic bearing frequencies can be calculated and monitored:

BPFO (Ball Pass Frequency Outer Race)
BPFI (Ball Pass Frequency Inner Race)
BSF (Ball Spin Frequency)
FTF (Fundamental Train Frequency)
Communication Interface

The firmware streams measurement and analysis results through UART for external visualization and monitoring.

Supported packet types include:

RAW
FFT
SPEC
FAULT_FREQ
System Architecture
LIS3DSH Accelerometer
          │
          ▼
    STM32F407 MCU
          │
          ▼
   CMSIS-DSP FFT
          │
          ▼
   UART Communication
          │
          ▼
 Qt Desktop Application
          │
          ▼
 Bearing Fault Analysis
Project Structure
Core/
Drivers/
Middlewares/
USB_HOST/
Debug/

STM32F407VGTX_FLASH.ld
STM32F407VGTX_RAM.ld
yeni_usart.ioc
FFT Configuration
FFT Size        : 512
Sampling Rate   : 1600 Hz
Maximum Frequency : 800 Hz (Nyquist Limit)
Build Instructions
Clone the repository.
git clone https://github.com/<username>/stm32-cnc-vibration-monitor.git
Open the project in STM32CubeIDE.
Generate code if required using:
yeni_usart.ioc
Build the project.
Flash the firmware to STM32F407 Discovery.
Connect the board via UART and start the desktop monitoring application.
Companion Desktop Application

This repository contains only the embedded firmware.

The desktop visualization software is maintained in a separate repository:

stm32-cnc-vibration-monitor   → STM32 Firmware
stm32-uart-reader             → Qt Desktop Application
Current Status
UART communication operational
LIS3DSH sensor integration completed
Real-time vibration acquisition completed
CMSIS-DSP integration completed
FFT infrastructure completed
Qt communication protocol defined
Future Improvements
DMA-based acquisition
Higher sampling rate support
Envelope analysis
Crest Factor and Kurtosis calculations
Data logging
Fault scoring algorithms
Machine learning based classification
Predictive maintenance features
License

This project is developed for educational, research, and condition monitoring applications.

Author: Melih Fındık
Platform: STM32F407 Discovery
Application Domain: CNC Spindle & Bearing Condition Monitoring

## Development Status

Current repository contains only the STM32 firmware side of the project.

The Qt-based desktop monitoring application will be published as a separate repository in future versions.
