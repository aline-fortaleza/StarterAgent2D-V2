// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

// Student Soccer 2D Simulation Base , STDAGENT2D
// Simplified the Agent2D Base for HighSchool Students.
// Technical Committee of Soccer 2D Simulation League, IranOpen
// Nader Zare
// Mostafa Sayahi
// Pooria Kaviani
/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_offensive_kick.h"

#include "basic_actions/body_hold_ball.h"
#include "basic_actions/body_smart_kick.h"
#include "basic_actions/body_stop_ball.h"
#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>


#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <rcsc/geom/sector_2d.h>

#include <vector>
using namespace rcsc;



/*!
      \brief check if a player is in the region
      \param wm WorldModel
      \param region usually the pass cone defined in the pass function
      \return true or false
    */
bool Is_inRegion(const WorldModel &wm, const Sector2D &region, const AbstractPlayerObject *opponent) 
{
    if (region.contains(opponent->pos()))
    {
        return true;
    }
    return false;
}

/*!
      \brief count how many players is in the region
      \param wm WorldModel
      \param region usually the pass cone defined in the pass function
      \return number of players in the region
    */
int countOpponentsInRegion(const WorldModel &wm, const Sector2D &region) 
{
    Vector2D ball_pos = wm.ball().pos();
    int count = 0;
    for (int u = 1; u <= 11; u++)
    {
        const AbstractPlayerObject *tm = wm.theirPlayer(u);
        if (tm == NULL || tm->unum() < 1 )
            continue;
        Vector2D tm_pos = tm->pos();
        if (tm->pos().dist(ball_pos) > 30)
            continue;
        if (Is_inRegion(wm, region, tm))
        {
            count++;
        }
    }
    return count;
}

/*!
      \brief Distance from a player to the pass line
      \param wm WorldModel
      \param player player being analyzed now
      \param opponent opponent being analyzed now
      \return distance from actual opponent to the pass line
    */
double pointToLine(const WorldModel &wm, Vector2D& player, const AbstractPlayerObject *opponent){
    Vector2D ball_pos = wm.ball().pos();
    Vector2D opp_pos = opponent->pos();

    // Calcula a distância do oponente à linha que liga a bola ao jogador
    double numerador = std::fabs((player.y - ball_pos.y) * opp_pos.x
                               - (player.x - ball_pos.x) * opp_pos.y
                               + player.x * ball_pos.y
                               - player.y * ball_pos.x);

    double denominador = std::sqrt(std::pow(player.y - ball_pos.y, 2)
                                 + std::pow(player.x - ball_pos.x, 2));

    if (denominador == 0.0) return 0.0; // evita divisão por zero

    return numerador / denominador;
    
}


/*!
      \brief Nearest Opponent Distance From Pass Line
      \param wm WorldModel
      \param player player being analyzed now
      \return distance from nearest opponent to the pass line
    */
double Nearest_Opponent_Line(const WorldModel &wm, Vector2D& player){

    Vector2D ball_pos = wm.ball().pos();
    int nearest_until_now = 30; // valor arbitrário 
    for (int u = 1; u<=11; u++){
        const AbstractPlayerObject *opponent = wm.theirPlayer(u);
        if (opponent == NULL || opponent->unum() < 1)
            continue;
        Vector2D opp_pos = opponent->pos();
        if (opp_pos.dist(ball_pos) > 30)
            continue;
        if ( pointToLine(wm, player, opponent) < nearest_until_now)
        {
            nearest_until_now = pointToLine(wm, player, opponent);
        }
    }
    return nearest_until_now;

}

/*!
      \brief Nearest Opponent Distance From player
      \param wm WorldModel
      \param player player being analyzed now
      \return distance from nearest opponent to the player
    */
double Nearest_Opponent (const WorldModel &wm, Vector2D& player){
    Vector2D ball_pos = wm.ball().pos();
    int nearest_until_now = 30; // valor arbitrário 
    for (int u = 1; u<=11; u++){
        const AbstractPlayerObject *opponent = wm.theirPlayer(u);
        if (opponent == NULL || opponent->unum() < 1)
            continue;
        Vector2D opp_pos = opponent->pos();
        if (opp_pos.dist(ball_pos) > 30)
            continue;
        if (opp_pos.dist(player) < nearest_until_now)
        {
            nearest_until_now = opp_pos.dist(player);
        }
    }
    return nearest_until_now;
}

double Priority_pontuation (const WorldModel &wm, const Sector2D &region, PlayerAgent *agent, int Opponents_in_region, double nearest_opponent_line,  double nearest_opp_dist, Vector2D &player) {
    int qtd_opponents;
    Vector2D ball_pos = agent->world().ball().pos(); 
    int distance_to_ball = player.dist(ball_pos);

    double Pass_score = (20 - distance_to_ball*2.5) + (nearest_opponent_line*7) + (nearest_opp_dist*5.625) + (3 - Opponents_in_region)*3.25;
    
    return Pass_score;
}

/*-------------------------------------------------------------------*/
/*!

 */
bool Bhv_BasicOffensiveKick::execute(PlayerAgent *agent)
{
    dlog.addText(Logger::TEAM,
                 __FILE__ ": Bhv_BasicOffensiveKick");

    const WorldModel &wm = agent->world();

    if (shoot(agent))
    {
        return true;
    }

    const auto &opps = wm.opponentsFromSelf();
    const PlayerObject *nearest_opp = (opps.empty()
                                           ? static_cast<PlayerObject *>(0)
                                           : opps.front());
    const double nearest_opp_dist = (nearest_opp
                                         ? nearest_opp->distFromSelf()
                                         : 1000.0);
    //    const Vector2D nearest_opp_pos = ( nearest_opp
    //                                       ? nearest_opp->pos()
    //                                       : Vector2D( -1000.0, 0.0 ) );

    if (nearest_opp_dist < 10)
    {
        if (pass(agent))
            return true;
    }

    if (dribble(agent))
    {
        return true;
    }

    if (nearest_opp_dist > 2.5)
    {
        dlog.addText(Logger::TEAM,
                     __FILE__ ": hold");
        agent->debugClient().addMessage("OffKickHold");
        Body_HoldBall().execute(agent);
        return true;
    }
    clearball(agent); // chutar pra fora
    return true;
}

bool Bhv_BasicOffensiveKick::shoot(rcsc::PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    Vector2D ball_pos = wm.ball().pos();
    Vector2D center_goal = Vector2D(52.5, 0);
    if (ball_pos.dist(center_goal) > 25)
        return false;
    Vector2D left_goal = Vector2D(52.5, 6);
    Vector2D right_goal = Vector2D(52.5, -6);

    if (left_goal.dist(ball_pos) < right_goal.dist(ball_pos))
    {
        Body_SmartKick(left_goal, 3, 0.1, 2).execute(agent);
    }
    else
    {
        Body_SmartKick(right_goal, 3, 0.1, 2).execute(agent);
    }
    return true;
}

bool Bhv_BasicOffensiveKick::pass(PlayerAgent *agent, int kick_count)
{
    const WorldModel &wm = agent->world();
    std::map<double, Vector2D, std::greater<int>> targets;
    Vector2D ball_pos = wm.ball().pos();
    for (int u = 1; u <= 11; u++)
    {
        const AbstractPlayerObject *tm = wm.ourPlayer(u);
        if (tm == NULL || tm->unum() < 1 || tm->unum() == wm.self().unum())
            continue;
        Vector2D tm_pos = tm->pos();
        if (tm->pos().dist(ball_pos) > 30)
            continue;
        Sector2D pass = Sector2D(ball_pos, 1, tm_pos.dist(ball_pos) + 3, (tm_pos - ball_pos).th() - 15, (tm_pos - ball_pos).th() + 15);

        //parametros 
        int d = countOpponentsInRegion(wm, pass);
        double od = Nearest_Opponent_Line(wm, tm_pos);
        double x = Nearest_Opponent(wm, tm_pos);

        //calcula o score
        double score = Priority_pontuation(wm, pass, agent, d, od, x, tm_pos);
        
        //targets.push_back({tm_pos, score})
        targets.insert({score, tm_pos});



        // if (!wm.existOpponentIn(pass, 5, true))
        // {
        //     targets.push_back(tm_pos);
        // }
    }
    if (targets.size() == 0)
        return false; 
    Vector2D best_target = targets.begin()-> second;
    // for (unsigned int i = 1; i < targets.size(); i++)
    // {
    //     if (targets[i].x > best_target.x)
    //         best_target = targets[i];
    // }
    if (wm.gameMode().type() != GameMode::PlayOn)
        Body_SmartKick(best_target, kick_count, 2.5, 1).execute(agent);
    else
        Body_SmartKick(best_target, kick_count, 2.5, 2).execute(agent);
    return true;
}


bool Bhv_BasicOffensiveKick::dribble(PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    Vector2D ball_pos = wm.ball().pos();
    double dribble_angle = (Vector2D(52.5, 0) - ball_pos).th().degree();
    Sector2D dribble_sector = Sector2D(ball_pos, 0, 3, dribble_angle - 15, dribble_angle + 15);
    if (!wm.existOpponentIn(dribble_sector, 5, true))
    {
        Vector2D target = Vector2D::polar2vector(3, dribble_angle) + ball_pos;
        if (Body_SmartKick(target, 0.8, 0.7, 2).execute(agent))
        {
            return true;
        }
    }
    return false;
}

bool Bhv_BasicOffensiveKick::clearball(PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    if (!wm.self().isKickable())
        return false;
    Vector2D ball_pos = wm.ball().pos();
    Vector2D target = Vector2D(52.5, 0);
    if (ball_pos.x < 0)
    {
        if (ball_pos.x > -25)
        {
            if (ball_pos.dist(Vector2D(0, -34)) < ball_pos.dist(Vector2D(0, +34)))
            {
                target = Vector2D(0, -34);
            }
            else
            {
                target = Vector2D(0, +34);
            }
        }
        else
        {
            if (ball_pos.absY() < 10 && ball_pos.x < -10)
            {
                if (ball_pos.y > 0)
                {
                    target = Vector2D(-52, 20);
                }
                else
                {
                    target = Vector2D(-52, -20);
                }
            }
            else
            {
                if (ball_pos.y > 0)
                {
                    target = Vector2D(ball_pos.x, 34);
                }
                else
                {
                    target = Vector2D(ball_pos.x, -34);
                }
            }
        }
    }
    if (Body_SmartKick(target, 2.7, 2.7, 2).execute(agent))
    {
        return true;
    }
    Body_StopBall().execute(agent);
    return true;
}