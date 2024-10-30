#include "ros/ros.h"
#include "my_playlist/CreatePlaylist.h"
#include "my_playlist/AddSongToPlaylist.h"
#include "my_playlist/ShowPlaylists.h"
#include "my_playlist/SwitchPlaylist.h"
#include "my_playlist/DeletePlaylist.h"
#include "my_playlist/DisplayPlaylistSongs.h"
#include "my_playlist/RemoveSongFromPlaylist.h"
#include "my_playlist/SortSongsByDuration.h"
#include <string>
#include <vector>

class PlaylistManager {
private:
    struct Song {
        std::string title;
        std::string artist;
        std::string genre;
        double duration;
    };
    struct Playlist {
        std::string name;
        std::vector<Song> songs;
    };

    std::vector<Playlist> playlists;
    int currentPlaylistIndex = -1;

public:
    bool createPlaylist(playlist_manager::CreatePlaylist::Request &req,
                        playlist_manager::CreatePlaylist::Response &res) {
        playlists.push_back({req.name, {}});
        currentPlaylistIndex = playlists.size() - 1;
        res.success = true;
        res.message = "Playlist created and selected";
        return true;
    }

    bool addSongToPlaylist(playlist_manager::AddSongToPlaylist::Request &req,
                           playlist_manager::AddSongToPlaylist::Response &res) {
        if (currentPlaylistIndex == -1) {
            res.success = false;
            res.message = "No playlist selected";
            return true;
        }
        playlists[currentPlaylistIndex].songs.push_back({req.title, req.artist, req.genre, req.duration});
        res.success = true;
        res.message = "Song added to playlist";
        return true;
    }

    bool showPlaylists(playlist_manager::ShowPlaylists::Request &,
                       playlist_manager::ShowPlaylists::Response &res) {
        for (const auto &playlist : playlists) {
            res.playlist_names.push_back(playlist.name);
        }
        return true;
    }

    bool switchPlaylist(playlist_manager::SwitchPlaylist::Request &req,
                        playlist_manager::SwitchPlaylist::Response &res) {
        for (size_t i = 0; i < playlists.size(); ++i) {
            if (playlists[i].name == req.name) {
                currentPlaylistIndex = i;
                res.success = true;
                res.message = "Playlist switched";
                return true;
            }
        }
        res.success = false;
        res.message = "Playlist not found";
        return true;
    }

    bool deletePlaylist(playlist_manager::DeletePlaylist::Request &req,
                        playlist_manager::DeletePlaylist::Response &res) {
        for (auto it = playlists.begin(); it != playlists.end(); ++it) {
            if (it->name == req.name) {
                playlists.erase(it);
                currentPlaylistIndex = -1;
                res.success = true;
                res.message = "Playlist deleted";
                return true;
            }
        }
        res.success = false;
        res.message = "Playlist not found";
        return true;
    }

    bool displayPlaylistSongs(playlist_manager::DisplayPlaylistSongs::Request &,
                              playlist_manager::DisplayPlaylistSongs::Response &res) {
        if (currentPlaylistIndex == -1) {
            return true;
        }
        for (const auto &song : playlists[currentPlaylistIndex].songs) {
            res.song_details.push_back("Title: " + song.title + ", Artist: " + song.artist);
        }
        return true;
    }

    bool removeSongFromPlaylist(playlist_manager::RemoveSongFromPlaylist::Request &req,
                                playlist_manager::RemoveSongFromPlaylist::Response &res) {
        if (currentPlaylistIndex == -1) {
            res.success = false;
            res.message = "No playlist selected";
            return true;
        }
        auto &songs = playlists[currentPlaylistIndex].songs;
        for (auto it = songs.begin(); it != songs.end(); ++it) {
            if (it->title == req.title) {
                songs.erase(it);
                res.success = true;
                res.message = "Song removed";
                return true;
            }
        }
        res.success = false;
        res.message = "Song not found";
        return true;
    }

    bool sortSongsByDuration(playlist_manager::SortSongsByDuration::Request &,
                             playlist_manager::SortSongsByDuration::Response &res) {
        if (currentPlaylistIndex == -1) {
            res.success = false;
            res.message = "No playlist selected";
            return true;
        }
        auto &songs = playlists[currentPlaylistIndex].songs;
        std::sort(songs.begin(), songs.end(), [](const Song &a, const Song &b) {
            return a.duration < b.duration;
        });
        res.success = true;
        res.message = "Songs sorted by duration";
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "playlist_manager_node");
    ros::NodeHandle n;

    PlaylistManager manager;

    ros::ServiceServer createPlaylistService = n.advertiseService("create_playlist", &PlaylistManager::createPlaylist, &manager);
    ros::ServiceServer addSongService = n.advertiseService("add_song_to_playlist", &PlaylistManager::addSongToPlaylist, &manager);
    ros::ServiceServer showPlaylistsService = n.advertiseService("show_playlists", &PlaylistManager::showPlaylists, &manager);
    ros::ServiceServer switchPlaylistService = n.advertiseService("switch_playlist", &PlaylistManager::switchPlaylist, &manager);
    ros::ServiceServer deletePlaylistService = n.advertiseService("delete_playlist", &PlaylistManager::deletePlaylist, &manager);
    ros::ServiceServer displaySongsService = n.advertiseService("display_playlist_songs", &PlaylistManager::displayPlaylistSongs, &manager);
    ros::ServiceServer removeSongService = n.advertiseService("remove_song_from_playlist", &PlaylistManager::removeSongFromPlaylist, &manager);
    ros::ServiceServer sortSongsService = n.advertiseService("sort_songs_by_duration", &PlaylistManager::sortSongsByDuration, &manager);

    ROS_INFO("Playlist Manager Node Started");
    ros::spin();

    return 0;
}
