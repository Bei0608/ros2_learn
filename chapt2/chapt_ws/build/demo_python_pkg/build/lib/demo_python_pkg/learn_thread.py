import threading
import requests


class Download:
    def download(self, url, callback):
        print(f'线程:{threading.get_ident()} 开始下载 {url}')
        response  = requests.get(url)
        response.encoding = 'utf-8'
        callback(response.text)

    def start_download(self, url, callback):
        thread = threading.Thread(target=self.download, args=(url, callback))
        thread.start()
        
        
def download_finish_callback(url, result):
    print(f'线程:{threading.get_ident()} 下载 {url} 完成, 内容长度为 {len(result)} 内容为:{result[:5]}...')
    

def main():
    d = Download()
    d.start_download('https://www.example.com', lambda result: download_finish_callback('http://localhost:8000/novel1.txt', result))
    d.start_download('https://www.example.org', lambda result: download_finish_callback('http://localhost:8000/novel2.txt', result))