# Ubuntu 22.04 LTS WiFi熱點設定指南

## 步驟1：安裝必要軟體包

```bash
sudo apt update
sudo apt install hostapd dnsmasq -y
```

## 步驟2：停止服務並備份設定檔

```bash
sudo systemctl stop hostapd
sudo systemctl stop dnsmasq
sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.backup
sudo cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup
sudo cp /etc/hostapd/hostapd.conf /etc/hostapd/hostapd.conf.backup 2>/dev/null || true
```

## 步驟3：設定靜態IP

編輯網路設定，將WiFi介面設定為靜態IP 192.168.0.100：

```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

內容範例：
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    # 有線網路介面（如果有的話）
    eth0:  # 或是你的有線網路介面名稱
      dhcp4: true
  wifis:
    wlan0:  # 確認你的WiFi介面名稱
      access-points: {}
      addresses:
        - 192.168.0.100/24
```

套用設定：
```bash
sudo netplan apply
```

## 步驟4：設定dnsmasq（DHCP服務）

編輯dnsmasq設定檔：
```bash
sudo nano /etc/dnsmasq.conf
```

在檔案末尾加入：
```
interface=wlan0
dhcp-range=192.168.0.101,192.168.0.150,255.255.255.0,24h
dhcp-option=3,192.168.0.100
dhcp-option=6,8.8.8.8,8.8.4.4
```

## 步驟5：設定hostapd（WiFi熱點）

建立hostapd設定檔：
```bash
sudo nano /etc/hostapd/hostapd.conf
```

內容：
```
interface=wlan0
driver=nl80211
ssid=Ubuntu-Hotspot
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=YourPassword123
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```

設定hostapd使用此設定檔：
```bash
sudo nano /etc/default/hostapd
```

找到並修改：
```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

## 步驟6：啟用IP轉發

編輯sysctl設定：
```bash
sudo nano /etc/sysctl.conf
```

取消註解或加入：
```
net.ipv4.ip_forward=1
```

立即啟用：
```bash
sudo sysctl -w net.ipv4.ip_forward=1
```

## 步驟7：設定防火牆規則（NAT）

如果你想讓熱點用戶能上網，需要設定NAT規則：

```bash
# 安裝iptables-persistent來保存規則
sudo apt install iptables-persistent -y

# 設定NAT規則（假設你的網際網路連線是透過eth0）
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT

# 保存iptables規則
sudo sh -c "iptables-save > /etc/iptables/rules.v4"
```

## 步驟8：設定SSH服務

安裝並啟用SSH服務：
```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

檢查SSH服務狀態：
```bash
sudo systemctl status ssh
```

編輯SSH設定（可選，增強安全性）：
```bash
sudo nano /etc/ssh/sshd_config
```

建議的安全設定：
```
Port 22
PermitRootLogin no
PasswordAuthentication yes
PubkeyAuthentication yes
```

重啟SSH服務：
```bash
sudo systemctl restart ssh
```

## 步驟9：啟動服務

```bash
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
sudo systemctl start hostapd
sudo systemctl start dnsmasq
```

## 步驟10：驗證設定

檢查服務狀態：
```bash
sudo systemctl status hostapd
sudo systemctl status dnsmasq
```

檢查網路介面：
```bash
ip addr show wlan0
```

檢查是否有WiFi熱點：
```bash
iwconfig
```

## 客戶端連線方式

1. 在其他設備上搜尋WiFi網路 "Ubuntu-Hotspot"
2. 使用密碼 "YourPassword123" 連接
3. 連接後，設備會自動獲得192.168.0.101-192.168.0.150範圍的IP
4. 使用SSH連接Ubuntu主機：

```bash
ssh username@192.168.0.100
```

其中 `username` 是你的Ubuntu用戶名。

## 故障排除

如果遇到問題，可以檢查：

1. 檢查WiFi介面名稱：
```bash
ip link show
```

2. 檢查hostapd日誌：
```bash
sudo journalctl -u hostapd -f
```

3. 檢查dnsmasq日誌：
```bash
sudo journalctl -u dnsmasq -f
```

4. 確認防火牆沒有阻擋：
```bash
sudo ufw status
```

5. 手動測試hostapd：
```bash
sudo hostapd -d /etc/hostapd/hostapd.conf
```

## 注意事項

1. 確保你的WiFi網卡支援AP模式
2. 如果你的系統使用NetworkManager，可能需要額外設定
3. 記得更改預設密碼以提高安全性
4. 考慮設定防火牆規則限制SSH存取
