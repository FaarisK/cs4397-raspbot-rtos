import requests
import json
 
 
def run_dify_workflow(strQQ='Forward for 3 seconds, play music'):
    # 配置参数
    url = "http://localhost/v1/chat-messages"
    api_key = "app-iTlHn9ETR4LqqVFBBbAMZkum"  # 你的API密钥
    user_id = "abc-123"  # 用户标识
 
    # 请求头
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json;charset=UTF-8",
        "Accept-Charset": "UTF-8"
    }
 
    # 请求体
    payload = {
        "inputs": {},
        "query":strQQ,
        "response_mode": "blocking",# blocking
        "user": user_id
    }
 
    try:
        # 发送POST请求
        response = requests.post(
            url,
            headers=headers,
            json=payload,
            stream=False  
        )
        #print(f"请求成功，状态码: {response.status_code}")

        # 检查响应状态
        if response.status_code != 200:
            print(f"Request failed, status code: {response.status_code}")
            print(f"error: {response.text}")
            return

        
        if response.status_code == 200:
            #由于不是工作流，所以取出来的不一定是["answer"],方法都是一样的，到时候自己打印response.json()出来就知道了
            answer = response.json()["answer"]
            print(answer)
            if answer:
                return answer
            else:
                return ""
        else:  
            return 0
        

    except requests.exceptions.RequestException as e:
        print(f"请求发生错误: {e}")

 
 
if __name__ == "__main__":
    aaa=run_dify_workflow()
    print(aaa)