#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <functional>
#include <algorithm>
#include <fstream>
#include <random>

using namespace std;

typedef pair<int, int> P;
const int INF = 1e9;

struct edge{
    int from; //出発点
    int to; //到着点
    int d; //重み
    int link; //容量
    bool operator>(const edge &e) const{
        return link > e.link;
    }
};

class UnionFind{
    vector<int> parent; //各頂点の親頂点の番号
    vector<int> depth; //各頂点の根付き木の頂点数
    vector<int> siz; //グループのサイズ

    public:
        //初期化
        UnionFind(int n) : parent(n, -1), depth(n, 0), siz(n, 1){}

        //根を求める
        int root(int x){
            if(parent[x] == -1) return x; //ベースケース(xが根であればxを返す)
            else return parent[x] = root(parent[x]); //再帰処理(経路圧縮)
        }

        //別々の木をまとめる
        void unite(int x, int y){
            //x,yを根まで移動
            x = root(x);
            y = root(y);

            if(x == y) return;//すでに一致していれば何もしない

            //yとxを統合する
            if(depth[x] < depth[y]) swap(x, y); //y側のサイズが小さくなるよう調整(union by size)
            parent[y] = x;
            siz[x] += siz[y];
            if(depth[x] == depth[y]) depth[x]++;
        }

        bool same(int x, int y){
            return root(x) == root(y);
        }

        //xを含むグループのサイズ
        int size(int x){
            return siz[root(x)];
        }
};

//ヒープを使った高速なダイクストラ法
vector<int> dijkstra(int s, int g, vector<vector<edge>> &G){
    vector<int> dist(G.size() + 10, INF);
    dist[s] = 0;
    //ヒープの作成
    priority_queue<P, vector<P>, greater<P>> que;
    que.push(P(dist[s], s));

    //ダイクストラ法の反復を開始
    while(!que.empty()){
        int v = que.top().second;
        int w = que.top().first;
        que.pop();

        if(w > dist[v]) continue;
        //緩和を行う
        for(auto e : G[v]){
            if(e.link == 0) continue;
            if(dist[e.to] > dist[v] + e.d){
                dist[e.to] = dist[v] + e.d;
                que.push(P(dist[e.to], e.to));
            }
        }
    }

    vector<int> res;
    if(dist[g] == INF) return res;

    res.push_back(g);
    int node = g;

    //経路の記録
    while(node != s){
        for(auto e : G[node]){
            if(dist[e.to] == dist[e.from] - e.d){
                node = e.to;
                res.push_back(node);
                break;
            }
        }
    }
    reverse(res.begin(), res.end()); //start→goalの形にする
    return res;
}

vector<int> widest_path(int s, int g, vector<vector<edge>> &G_ref){
    vector<edge> edges;
    for(auto ve : G_ref){
        for(auto e : ve){
            edges.push_back(e);
        }
    }
    sort(edges.begin(), edges.end(), greater<edge>());
    vector<vector<edge>> G(edges.size() + 10);
    UnionFind ut(edges.size() + 10);
    for(auto e : edges){
        ut.unite(e.from, e.to);
        G[e.from].push_back(e);

        auto e_rev = e;
        e_rev.from = e.to;
        e_rev.to = e.from;

        G[e.to].push_back(e_rev);
        if(ut.same(s, g)) break;
    }
    return dijkstra(s, g, G);
}

double simurator(int n, //パラメタn
                 int max_v, //ノード数
                 int cnt, //通信回数
                 vector<vector<edge>> &G_ref, //実験に使うグラフ
                 int algo //0:最小ホップ経路を用いた固定経路方式, 1:最小ホップ経路を用いた要求時経路方式, 2:最大路を用いた固定経路方式, 3:最大路を用いた要求時経路方式
                 ){
    //乱数の設定
    random_device seed_gen;
    default_random_engine engine(seed_gen());
    uniform_int_distribution<> dist(0, max_v);

    queue<vector<int>> que;//通信状態の記録
    vector<vector<edge>> G;//グラフ
    int fail = 0; //通信の確立できなかった回数
    for(auto ve : G_ref) G.push_back(ve);
    for(int i = 0; i < cnt; i++){
        //通信の終了
        if(que.size() >= n){
            auto fin_res = que.front();
            que.pop();
            for(int i = 0; i + 1 < fin_res.size(); i++){
                int from = fin_res[i];
                int to = fin_res[i + 1];
                //無向グラフの両方向に対して処理を行う
                for(int j = 0; j < G[from].size(); j++){
                    if(G[from][j].to == to) G[from][j].link++;
                }
                for(int j = 0; j < G[to].size(); j++){
                    if(G[to][j].to == from) G[to][j].link++;
                }
            }
        }

        int start, goal;
        do{
            start = dist(engine);
            goal = dist(engine);
        }while(start == goal); //送受信ノードが一致した場合に処理をスタートとゴールを再設定する

        //通信の実行
        vector<int> res;
        //G:グラフの状況を更新するため要求時経路に用いる
        //G_ref:グラフの状況を更新しないため固定経路に用いる
        if(algo == 0) res = dijkstra(start, goal, G_ref); //最小ホップ経路を用いた固定経路
        else if(algo == 1) res = dijkstra(start, goal, G); //最小ホップを用いた要求時経路
        else if(algo == 2) res = widest_path(start, goal, G_ref); //最大路を用いた固定経路
        else if(algo == 3) res = widest_path(start, goal, G); //最大路を用いた要求時経路
        for(int i = 0; i + 1 < res.size(); i++){
            int from = res[i];
            int to = res[i + 1];
            for(int j = 0; j < G[from].size(); j++){
                //通信に失敗した場合の処理
                if(G[from][j].to == to && G[from][j].link == 0) res.clear();
            }
        }
        for(int i = 0; i + 1 < res.size(); i++){
            int from = res[i];
            int to = res[i + 1];
            //通信に成功した場合、空き容量を１減らす
            for(int j = 0; j < G[from].size(); j++){
                if(G[from][j].to == to) G[from][j].link--;
            }
            //逆方向
            for(int j = 0; j < G[to].size(); j++){
                if(G[to][j].to == from) G[to][j].link--;
            }
        }
        if(res.size() < 2) fail += 1;
        que.push(res);
    }
    return fail / (double)cnt; //呼損率
}

int main(){
    int max_v = 0; //ノードの個数
    vector<edge> edges;
    vector<vector<edge>> Graph(100); //グラフ
    int from, to, dist, link; 
    ifstream fin("distance.txt"); //同じディレクトリにdistance.txt(名称は何でも可)を配置。
    if(!fin){
        fprintf(stderr, "ファイルが見つかりません\n");
        return 1;
    }
    while(!fin.eof()){
        fin >> from >> to >> dist >> link;

        edge e = edge{from, to, dist, link};
        max_v = max(max(max_v, from), to);
        edges.push_back(e);

        //双方向のグラフを作成
        Graph[from].push_back(e);
        swap(e.from, e.to);
        Graph[to].push_back(e);
    }
    cout << "最小ホップ経路を用いた固定経路方式" << endl;
    int base = 1;
    for(int i = 0; i <= 3; i++){
        for(int j = 1; j < 10; j++){
            //simulator(パラメタn, ノード数, 通信回数, 実験に使うグラフ, 利用する経路)
            auto res = simurator(base * j, max_v, 10000, Graph, 0);
            //cout << base * j << " : " << res << endl;
            cout << res << endl;
        }
        
        base *= 10;
    }
    auto res = simurator(base, max_v, 10000, Graph, 0);
    cout << res << endl;
    cout << endl;
    cout << "最小ホップ経路を用いた要求時経路方式" << endl;
    base = 1;
    for(int i = 0; i <= 3; i++){
        for(int j = 1; j < 10; j++){
            //simulator(パラメタn, ノード数, 通信回数, 実験に使うグラフ, 利用する経路)
            auto res = simurator(base * j, max_v, 10000, Graph, 1);
            //cout << base * j << " : " << res << endl;
            cout << res << endl;
        }
        base *= 10;
    }
    res = simurator(base, max_v, 10000, Graph, 0);
    cout << res << endl;
    cout << endl;
    cout << "最大路を用いた固定経路方式" << endl;
    base = 1;
    for(int i = 0; i <= 3; i++){
        for(int j = 1; j < 10; j++){
            //simulator(パラメタn, ノード数, 通信回数, 実験に使うグラフ, 利用する経路)
            auto res = simurator(base * j, max_v, 10000, Graph, 2);
            //cout << base * j << " : " << res << endl;
            cout << res << endl;
        }
        base *= 10;
    }
    res = simurator(base, max_v, 10000, Graph, 0);
    cout << res << endl;
    cout << endl;
    cout << "最大路を用いた要求時経路方式" << endl;
    base = 1;
    for(int i = 0; i <= 3; i++){
        for(int j = 1; j < 10; j++){
            //simulator(パラメタn, ノード数, 通信回数, 実験に使うグラフ, 利用する経路)
            auto res = simurator(base * j, max_v, 10000, Graph, 3);
            //cout << base * j << " : " << res << endl;
            cout << res << endl;
        }
        base *= 10;
    }
    res = simurator(base, max_v, 10000, Graph, 0);
    cout << res << endl;
    cout << "finish:正常に終了" << endl;
    return 0;
}
